import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf_transformations


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # declare parameters
        self.declare_parameter('max_speed', 0.15)
        self.declare_parameter('max_turn_rate', 1.5)
        self.declare_parameter('is_active', True)

        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn_rate = self.get_parameter('max_turn_rate').value
        self.is_active = self.get_parameter('is_active').value
        self.get_logger().info(f'Parameters loaded: max_speed={self.max_speed}, max_turn_rate={self.max_turn_rate}, is_active={self.is_active}')

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.groundtruth_sub = self.create_subscription(Odometry, '/ground_truth', self.groundtruth_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # store latest closest obstacle info
        self.closest_range_front = float('inf')
        self.last_scan_ranges = None

        # Add timestamp tracking for odometry
        self.last_odom_time = None
        self.odom_timeout = 0.5  # seconds

        # avoid wall: FORWARD or TURN
        self.state = 'FORWARD'
        self.yaw = None
        self.turn_target_yaw = None
        self.obstacle_threshold = 0.3
        self.turn_tolerance = 0.1  # rad
        self.post_turn_deadline = 0.0

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat_list)

        # Track when we last received odometry
        self.last_odom_time = self.get_clock().now()

    def scan_callback(self, msg):
        # keep the latest full ranges for side analysis
        self.last_scan_ranges = msg.ranges

        # front sector: indices 0-15 and 345-359 (assuming 360 points)
        # be robust to different sizes: pick roughly +/- 15 degrees around 0
        n = len(msg.ranges)
        sector = 15  # +/-15 indices
        front_indices = list(range(0, sector + 1)) + list(range(n - sector, n))
        front_sector = [msg.ranges[i] for i in front_indices]

        valid_ranges = [r for r in front_sector if math.isfinite(r) and r > msg.range_min]
        self.closest_range_front = min(valid_ranges) if valid_ranges else float('inf')

        # enforce a waiting period after finishing a turn
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self.post_turn_deadline:
            return

        # trigger a turn if obstacle in front
        if self.state == 'FORWARD' and self.closest_range_front < self.obstacle_threshold:
            # decide clearest side
            side = self._determine_clearest_side(msg.ranges)
            if self.yaw is None:
                # cannot set a target without current yaw; stop and wait
                self.state = 'FORWARD'
                return

            # choose 90-degree turn toward the clearest side
            if side == 'LEFT':
                target = self.yaw + math.pi / 2.0
            else:
                target = self.yaw - math.pi / 2.0

            self.turn_target_yaw = self._snap_to_cardinal_yaw(target)
            self.state = 'TURN'
            self.get_logger().info(f'Starting TURN toward {side}, target_yaw={self.turn_target_yaw:.3f}')

    def _determine_clearest_side(self, ranges):
        if not ranges:
            return 'LEFT'  # default

        n = len(ranges)
        # define left sector (approx 45-135 deg) and right sector (225-315 deg)
        left_start = int(n * 45 / 360)
        left_end = int(n * 135 / 360)
        right_start = int(n * 225 / 360)
        right_end = int(n * 315 / 360)

        left_ranges = ranges[left_start:left_end + 1]
        right_ranges = ranges[right_start:right_end + 1]

        def avg_valid(r_list):
            vals = [r for r in r_list if math.isfinite(r) and r > 0.0]
            return sum(vals) / len(vals) if vals else 0.0

        left_avg = avg_valid(left_ranges)
        right_avg = avg_valid(right_ranges)

        return 'LEFT' if left_avg >= right_avg else 'RIGHT'

    def _snap_to_cardinal_yaw(self, yaw):
        # normalize yaw then snap to nearest multiple of 90 degrees
        y = self._normalize_angle(yaw)
        # multiples of pi/2
        candidates = [i * (math.pi / 2.0) for i in range(-2, 3)]
        best = min(candidates, key=lambda c: abs(self._shortest_angular_dist(y, c)))
        return self._normalize_angle(best)

    def timer_callback(self):
        cmd = Twist()
        if not self.is_active:
            self.publisher_.publish(cmd)
            return

        # stop if odom stale
        if self.last_odom_time is None:
            self.publisher_.publish(cmd)
            return
        elapsed = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        if elapsed > self.odom_timeout:
            self.publisher_.publish(cmd)
            return

        if self.state == 'FORWARD':
            cmd.linear.x = self.max_speed
            cmd.angular.z = 0.0
            self.publisher_.publish(cmd)
            return

        if self.state == 'TURN':
            if self.yaw is None or self.turn_target_yaw is None:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.publisher_.publish(cmd)
                return

            # compute shortest angular error (target - current)
            error = self._shortest_angular_dist(self.yaw, self.turn_target_yaw)
            # proportional controller
            k_p = 1.0
            ang = max(-self.max_turn_rate, min(self.max_turn_rate, k_p * error))

            # publish rotational command only
            cmd.linear.x = 0.0
            cmd.angular.z = ang
            self.publisher_.publish(cmd)

            # check completion
            if abs(error) < self.turn_tolerance:
                # finished turn
                self.state = 'FORWARD'
                self.turn_target_yaw = None
                # give a short drive-forward grace period
                self.post_turn_deadline = (self.get_clock().now().nanoseconds / 1e9) + 1.0
                self.get_logger().info('Turn complete, returning to FORWARD')

    def groundtruth_callback(self, msg):
        # not used in control, but keep timestamp if needed
        pass

    def _normalize_angle(self, a):
        # normalize to [-pi, pi)
        return math.atan2(math.sin(a), math.cos(a))

    def _shortest_angular_dist(self, from_angle, to_angle):
        # return minimal signed angle to get from 'from_angle' to 'to_angle'
        a = self._normalize_angle(to_angle) - self._normalize_angle(from_angle)
        return self._normalize_angle(a)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

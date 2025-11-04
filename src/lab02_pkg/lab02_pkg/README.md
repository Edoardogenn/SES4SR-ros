# Lab02

[02_Simulation and Visualization.pdf](Lab02/02_Simulation_and_Visualization.pdf)

---

## Installation

- Gazebo

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros

source /usr/share/gazebo/setup.bash # paste the following line on ~/.bashrc 
```

- Rviz

```bash
sudo apt update
sudo apt install ros-humble-rviz2
```

---

## turtlebot3_simulations package

```bash
# in the src directory
git clone https://github.com/SESASR-Course/turtlebot3_simulations.git

# in the workspace ws directory
sudo rosdep init
rosdep update
rosdep install --from-path src --ignore-src -y -r
colcon build --symlink-install

# launch gazebo and rviz
ros2 launch turtlebot3_gazebo lab02.launch.py
ros2 launch turtlebot3_bringup rviz2.launch.py
```

- Results:

![image.png](Lab02/image.png)

---

# EXERCISE

## Controller

Install transformation package to use tf 

```bash
sudo apt install ros-humble-tf-transformations
pip install --upgrade transforms3d
```

- Algorithm

![image.png](Lab02/image%201.png)

- Code

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf_transformations 
import math 

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # declare parameters
        self.declare_parameter('max_speed', 0.22)
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

        # avoid wall: FORWARD or TURN
        self.state = 'FORWARD'
        self.yaw = None
        self.turn_target_yaw = None
        self.obstacle_threshold = 0.5
        self.turn_tolerance = 0.04
        self.post_turn_deadline = 0.0

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation         # Use tf_transformations 
        quat_list = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat_list)         # Roll, Pitch, Yaw -> we only take yaw

    def scan_callback(self, msg):
        
        # 1 Check only the front sector for obstacles -> LiDar -> assuming 360 points, index 0 is forward.
        front_sector = msg.ranges[:16] + msg.ranges[345:] # Indices 0-15 and 345-359
        
        valid_ranges = [r for r in front_sector if math.isfinite(r) and r > msg.range_min]  # exclude invalid values (inf, nan, and below min range)
        
        self.closest_range_front = min(valid_ranges) if valid_ranges else float('inf')
        
        # enforce a waiting period to drive forward before allowing a new turn
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self.post_turn_deadline:
            return
            

        # 2 TURN if an obstacle is detected while FORWARD
        if self.state == 'FORWARD' and self.closest_range_front < self.obstacle_threshold:
            if self.yaw is None:
                self.get_logger().info('Obstacle detected but no odom yet — stopping')
                self.publisher_.publish(Twist())
                return
            
            turn_direction = self._determine_clearest_side(msg.ranges)    # Determine the clearest side for a 90-degree turn
            
            nominal_target = self.yaw + (math.pi / 2.0 * turn_direction)     # Calculate the nominal 90-degree turn
            self.turn_target_yaw = self._snap_to_cardinal_yaw(nominal_target) # avoid drift by snapping to cardinal directions

            self.state = 'TURN'
            
            direction_str = "LEFT (+90 deg)" if turn_direction == 1 else "RIGHT (-90 deg)"
            self.get_logger().warn(
                f'Triggering TURN: Front closest={self.closest_range_front:.2f}m. Turning {direction_str} to target_yaw={self.turn_target_yaw:.2f} rad'
            )
            

    def _determine_clearest_side(self, ranges):
        left_ranges = ranges[45:136]         # 90-degree sector Left (approx 45 to 135 degrees)
        right_ranges = ranges[225:316]         # 90-degree sector Right (approx 225 to 315 degrees)
        
        MAX_RANGE = 3.5         # Get the max range value (3.5 m)

        def get_valid_ranges(r_list):
            return [MAX_RANGE if math.isinf(r) else r for r in r_list if math.isfinite(r) or math.isinf(r)]   # MAX_RANGE if inf 
             
        def get_avg(r_list):
            valid_list = get_valid_ranges(r_list)
            
            if not valid_list:
                return 0.0
                
            return sum(valid_list) / len(r_list)             # Calculate average based on the length of the original sector (91 elements)

        left_avg = get_avg(left_ranges)
        right_avg = get_avg(right_ranges)
        
        return 1 if left_avg >= right_avg else -1  # 1 for left, -1 for right
    

    def _snap_to_cardinal_yaw(self, yaw):
        
        normalized_yaw = self._normalize_angle(yaw)         # Normalize to [-pi, pi] first
        step = math.pi / 2.0         # Cardinal directions are multiples of pi/2 (~1.5708 rad)
        
        N = round(normalized_yaw / step)         # Calculate steps (N) of pi/2 away we are from 0
        snapped_yaw = N * step         # Snap the yaw to the nearest multiple of step

        return self._normalize_angle(snapped_yaw)

    def timer_callback(self):
        msg = Twist()
        if not self.is_active:
            self.publisher_.publish(msg)
            return

        if self.state == 'FORWARD':             # Drive straight until obstacle is detected
            msg.linear.x = float(self.max_speed) 
            msg.angular.z = 0.0

        elif self.state == 'TURN':
            if self.turn_target_yaw is None or self.yaw is None:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                diff = self._shortest_angular_dist(self.yaw, self.turn_target_yaw)                 # Compute shortest angular difference
                ang = max(-float(self.max_turn_rate), min(float(self.max_turn_rate), 2.0 * diff))                 # Proportional control (P-gain = 2.0)
                
                if abs(diff) < self.turn_tolerance:                 # If within tolerance, finish turn and resume forward
                    self.state = 'FORWARD'
                    self.turn_target_yaw = None
                    
                    now = self.get_clock().now().nanoseconds / 1e9
                    self.post_turn_deadline = now + 0.5
                    
                    msg.linear.x = float(self.max_speed)
                    msg.angular.z = 0.0
                    self.get_logger().info('Turn complete, starting FORWARD movement.')
                else:                     # Continue rotation
                    msg.linear.x = 0.0
                    msg.angular.z = ang
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)
        self.get_logger().debug(f'Controller state={self.state}, cmd linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')

    
    def groundtruth_callback(self, msg):
        pass    

    def _normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def _shortest_angular_dist(self, from_angle, to_angle):
        diff = self._normalize_angle(to_angle - from_angle)
        return diff

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

- Terminal outcome

![image.png](Lab02/image%202.png)

---

## Params

```yaml
controller:
  ros_parameters:
    linear_speed: 0.22
    max_turn_rate: 1.5
    is_active: true
```

---

## Launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription              
from launch.launch_description_sources import PythonLaunchDescriptionSource 

def generate_launch_description():

    # 1. Azione per includere l'altro file launch
    gazebo_node = IncludeLaunchDescription( # #
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'), 
                'launch',
                'lab02.launch.py' # import external launch file
            ])
        ])
    )

    # Our controller node
    controller_node = Node(
        package='lab02_pkg',
        namespace='controller1',
        executable='controller',
        name='controller',
        parameters=[PathJoinSubstitution([
            FindPackageShare('lab02_pkg'), 'params', 'params.yaml'
        ])] 
    )

    return LaunchDescription([
        gazebo_node,            # start gazebo
        controller_node       # start controller
    ])
```

---

## Register, plot, and evaluate

- Recording

[gazeboMod.m4v](Lab02/gazeboMod.m4v)

[gazeborvizMod.m4v](Lab02/gazeborvizMod.m4v)

—

- Rqt_graph

![image.png](Lab02/image%203.png)

—

- Rosbag

```bash
ros2 bag record -a
```

It will create a metadata.yaml where all the info related to the active topics are stored

```yaml
rosbag2_bagfile_information:
  version: 5
  storage_identifier: sqlite3
  duration:
    nanoseconds: 271465138269
  starting_time:
    nanoseconds_since_epoch: 1761577453358179313
  message_count: 73730
  topics_with_message_count:
    - topic_metadata:
        name: /rosout
        type: rcl_interfaces/msg/Log
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 1\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 10\n    nsec: 0\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 56
    - topic_metadata:
        name: /imu
        type: sensor_msgs/msg/Imu
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 40808
    - topic_metadata:
        name: /odom
        type: nav_msgs/msg/Odometry
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 6030
    - topic_metadata:
        name: /events/write_split
        type: rosbag2_interfaces/msg/WriteSplitEvent
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 0
    - topic_metadata:
        name: /robot_description
        type: std_msgs/msg/String
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 1\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 1
    - topic_metadata:
        name: /tf_static
        type: tf2_msgs/msg/TFMessage
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 1\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 1
    - topic_metadata:
        name: /tf
        type: tf2_msgs/msg/TFMessage
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 10010
    - topic_metadata:
        name: /clock
        type: rosgraph_msgs/msg/Clock
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 2\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 2053
    - topic_metadata:
        name: /ground_truth
        type: nav_msgs/msg/Odometry
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 4098
    - topic_metadata:
        name: /parameter_events
        type: rcl_interfaces/msg/ParameterEvent
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 56
    - topic_metadata:
        name: /cmd_vel
        type: geometry_msgs/msg/Twist
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 2394
    - topic_metadata:
        name: /performance_metrics
        type: gazebo_msgs/msg/PerformanceMetrics
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 1168
    - topic_metadata:
        name: /joint_states
        type: sensor_msgs/msg/JointState
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 6030
    - topic_metadata:
        name: /scan
        type: sensor_msgs/msg/LaserScan
        serialization_format: cdr
        offered_qos_profiles: "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
      message_count: 1025
  compression_format: ""
  compression_mode: ""
  relative_file_paths:
    - rosbag2_2025_10_27-16_04_13_0.db3
  files:
    - path: rosbag2_2025_10_27-16_04_13_0.db3
      starting_time:
        nanoseconds_since_epoch: 1761577453358179313
      duration:
        nanoseconds: 271465138269
      message_count: 73730
```

- Plot it then on plotjupiter

![image.png](Lab02/image%204.png)

---

---

# Appendix

## LiDAR

- Docs → https://docs.ros.org/en/humble/p/sensor_msgs/msg/LaserScan.html
- message published → sensor_msgs/msg/LaserScan

```xml
---
header:
 stamp:
 sec: 15
 nanosec: 578000000
 frame_id: base_footprint
angle_min: 0.0
angle_max: 6.28318977355957
angle_increment: 0.017501922324299812
time_increment: 0.0
scan_time: 0.0
range_min: 0.11999999731779099
range_max: 3.5
ranges: '<sequence type: float, length: 360>'
intensities: '<sequence type: float, length: 360>'
---
```

---

## Setup.py

```python
from setuptools import find_packages, setup
import os           
from glob import glob 

package_name = 'lab02_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),         # Install ament resource index marker for package discovery
        ('share/' + package_name, ['package.xml']),         # Install package.xml metadata file
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))), # Install all Python launch files from the local launch/ directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu-gigi',
    maintainer_email='gigiomuratore@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = lab02_pkg.controller:main',
        ],
    },
)
```

## Package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lab02_pkg</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="gigiomuratore@gmail.com">ubuntu-gigi</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

```
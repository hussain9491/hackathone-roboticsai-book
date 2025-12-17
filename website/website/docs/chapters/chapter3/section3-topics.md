---
sidebar_position: 3
title: "Topics"
---

# Topics

## Introduction

Topics are the fundamental communication mechanism in ROS 2 that enable asynchronous, many-to-many data exchange between nodes. Unlike services which provide synchronous request-response communication, topics allow nodes to publish data continuously while other nodes subscribe to receive that data. This publish-subscribe pattern is essential for building reactive, event-driven robotic systems where sensors publish data streams and multiple consumers process that information simultaneously. Understanding topics is crucial for developing distributed robotic applications, especially in complex systems like humanoid robots where multiple subsystems need to share sensor data, control commands, and state information efficiently.

## 1. Defining ROS 2 Topics

### What Are Topics?

Topics in ROS 2 serve as named buses over which nodes exchange messages. They implement a publish-subscribe communication pattern that enables:

- **Data Streaming**: Continuous flow of information like sensor readings, camera feeds, or robot states
- **Decoupling**: Publishers and subscribers don't need direct knowledge of each other
- **Scalability**: Multiple publishers and subscribers can operate on the same topic
- **Asynchronous Communication**: Senders and receivers operate independently without blocking

### Topic Characteristics

#### Named Communication Channels
- **Unique Identifiers**: Each topic has a globally unique name (e.g., `/sensor_data`, `/cmd_vel`)
- **Message Type Consistency**: All publishers and subscribers on a topic must use the same message type
- **Discovery Mechanism**: Nodes automatically discover available topics through DDS (Data Distribution Service)

#### Message Types
Topics use standardized message types defined in `.msg` files that specify the data structure:
```
# Sensor readings message
float64 temperature
float64 humidity
uint8[] sensor_ids
geometry_msgs/Point position
```

#### Quality of Service (QoS) Settings
Each topic can be configured with QoS policies that define reliability, durability, and other communication characteristics.

### Use Cases in Robotics
- **Sensor Data Distribution**: Camera images, LiDAR scans, IMU readings
- **Control Commands**: Velocity commands, joint positions, actuator controls
- **State Information**: Robot pose, battery levels, system status
- **Event Notifications**: Obstacle detection, task completion, error conditions

## 2. Publisher (The Writer)

### Creating a Publisher Node

A publisher node creates and sends messages to a specific topic. Here's how to implement a publisher in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')

        # Create a publisher for the temperature topic
        self.publisher_ = self.create_publisher(
            msg_type=String,
            topic='/temperature_data',
            qos_profile=10  # QoS queue size
        )

        # Timer to publish data periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Create and populate the message
        msg = String()
        msg.data = f'Temperature: {20.0 + math.sin(self.i * 0.1) * 5.0:.2f}°C'

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    temperature_publisher = TemperaturePublisher()

    try:
        rclpy.spin(temperature_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        temperature_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Publisher Features

#### Custom Message Publishing
```python
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10  # Queue size
        )

    def send_velocity_command(self, linear_x, angular_z):
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_x
        cmd_msg.angular.z = angular_z

        self.cmd_vel_pub.publish(cmd_msg)
```

#### Multiple Publishers in One Node
```python
class MultiPublisherNode(Node):
    def __init__(self):
        super().__init__('multi_publisher')

        # Multiple publishers for different topics
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
```

### Publisher Best Practices
- **Appropriate QoS Settings**: Match QoS policies to your application's requirements
- **Message Rate Management**: Avoid overwhelming subscribers with excessive data rates
- **Error Handling**: Check if publishers are active before sending messages
- **Resource Cleanup**: Properly destroy publishers when shutting down

## 3. Subscriber (The Listener)

### Creating a Subscriber Node

A subscriber node receives messages from a specific topic. Here's how to implement a subscriber in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')

        # Create a subscription to the temperature topic
        self.subscription = self.create_subscription(
            String,
            '/temperature_data',
            self.listener_callback,
            10  # QoS queue size
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received temperature: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    temperature_subscriber = TemperatureSubscriber()

    try:
        rclpy.spin(temperature_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        temperature_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Subscriber Features

#### Processing Complex Messages
```python
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Process the image (example: edge detection)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        self.get_logger().info(f'Processed image: {cv_image.shape}')
```

#### Multiple Subscriptions in One Node
```python
class MultiSubscriberNode(Node):
    def __init__(self):
        super().__init__('multi_subscriber')

        # Multiple subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

    def odom_callback(self, msg):
        self.get_logger().info(f'Position: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})')

    def scan_callback(self, msg):
        min_range = min(msg.ranges)
        self.get_logger().info(f'Min obstacle distance: {min_range}')

    def imu_callback(self, msg):
        orientation = msg.orientation
        self.get_logger().info(f'Orientation: ({orientation.x}, {orientation.y}, {orientation.z})')
```

### Subscriber Best Practices
- **Efficient Callbacks**: Keep callback functions lightweight to avoid blocking
- **Threading Considerations**: Be aware of thread safety when accessing shared data
- **Queue Management**: Set appropriate queue sizes to balance responsiveness and memory usage
- **Data Validation**: Validate incoming messages before processing

## 4. Quality of Service (QoS)

### QoS Overview

Quality of Service (QoS) profiles in ROS 2 allow fine-tuning of communication behavior to match application requirements. QoS settings determine how messages are delivered between publishers and subscribers.

### Key QoS Policies

#### Reliability Policy
Controls whether messages are guaranteed to be delivered:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Reliable delivery (messages will be retried if lost)
reliable_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE
)

# Best effort (no retries, faster but may lose messages)
best_effort_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT
)

# Using QoS in publisher/subscriber
publisher = node.create_publisher(String, '/reliable_topic', reliable_qos)
subscriber = node.create_subscription(String, '/best_effort_topic', callback, best_effort_qos)
```

#### Durability Policy
Determines how messages are handled for late-joining subscribers:

```python
from rclpy.qos import DurabilityPolicy

# Transient local (keep messages for new subscribers)
transient_qos = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

# Volatile (don't keep messages for new subscribers)
volatile_qos = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)
```

#### History Policy
Controls how many samples are stored:

```python
from rclpy.qos import HistoryPolicy

# Keep last N samples
keep_last_qos = QoSProfile(
    depth=5,  # Keep last 5 samples
    history=HistoryPolicy.KEEP_LAST
)

# Keep all samples (limited by resource limits)
keep_all_qos = QoSProfile(
    depth=0,  # Keep all samples (up to resource limits)
    history=HistoryPolicy.KEEP_ALL
)
```

### Practical QoS Examples

#### High-Frequency Sensor Data (Camera, LiDAR)
```python
# For high-frequency data where newer data is more valuable
sensor_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)
```

#### Critical Control Commands
```python
# For commands where delivery is critical
control_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)
```

#### Configuration Parameters (Static)
```python
# For static configuration that should persist
config_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST
)
```

### Matching QoS Between Publishers and Subscribers

For successful communication, publisher and subscriber QoS profiles must be compatible. The system will negotiate the most restrictive policy between them.

## 5. Practical Implementation Patterns

### Publisher-Subscriber Communication Pattern

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/joint_commands', 10)

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.current_joint_positions = {}

    def joint_state_callback(self, msg):
        """Callback to update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def control_loop(self):
        """Main control loop"""
        # Example: Send target joint positions
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = ['joint1', 'joint2', 'joint3']
        cmd_msg.position = [1.0, 0.5, -0.3]  # Target positions

        self.joint_cmd_pub.publish(cmd_msg)
```

### Bidirectional Communication Pattern

```python
class BidirectionalNode(Node):
    def __init__(self):
        super().__init__('bidirectional_comm')

        # Publisher and subscriber pair
        self.request_pub = self.create_publisher(String, '/requests', 10)
        self.response_sub = self.create_subscription(String, '/responses', self.response_callback, 10)

        # Timer to send requests periodically
        self.timer = self.create_timer(2.0, self.send_request)
        self.request_counter = 0

    def send_request(self):
        msg = String()
        msg.data = f'Request {self.request_counter}'
        self.request_pub.publish(msg)
        self.request_counter += 1

    def response_callback(self, msg):
        self.get_logger().info(f'Received response: {msg.data}')
```

### Message Filtering and Processing

```python
class MessageFilterNode(Node):
    def __init__(self):
        super().__init__('message_filter')

        # Subscribe to raw sensor data
        self.raw_sub = self.create_subscription(
            LaserScan, '/scan_raw', self.filter_scan, 10)

        # Publish filtered data
        self.filtered_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

    def filter_scan(self, msg):
        """Filter out invalid range values"""
        filtered_msg = msg
        filtered_ranges = []

        for range_val in msg.ranges:
            if 0.1 < range_val < 10.0:  # Valid range
                filtered_ranges.append(range_val)
            else:
                filtered_ranges.append(float('inf'))  # Invalid value

        filtered_msg.ranges = filtered_ranges
        self.filtered_pub.publish(filtered_msg)
```

## 6. Troubleshooting and Best Practices

### Common Issues and Solutions

#### Topic Connection Problems
- **Issue**: Publishers and subscribers not connecting
- **Solution**: Verify topic names match exactly (including leading `/`)
- **Debug**: Use `ros2 topic list` and `ros2 topic info` to verify connections

#### Performance Issues
- **Issue**: High CPU usage or delayed messages
- **Solution**: Optimize QoS settings and message rates
- **Debug**: Use `ros2 topic hz` to check message frequency

#### Memory Issues
- **Issue**: Memory leaks with large message queues
- **Solution**: Use appropriate queue depths and cleanup policies

### Best Practices

#### Naming Conventions
- Use descriptive, consistent names: `/sensor_data/lidar_scan` vs `/scan`
- Follow namespace conventions: `/robot1/sensors/imu` for multi-robot systems
- Avoid special characters except underscores and forward slashes

#### Message Design
- Keep messages appropriately sized (avoid extremely large messages)
- Use appropriate data types for the application
- Include timestamps when temporal information is important

#### Performance Optimization
- Match QoS settings to application requirements
- Use appropriate message rates (not too frequent, not too sparse)
- Consider message compression for large data like images

#### Testing and Debugging
```bash
# List all topics
ros2 topic list

# Get information about a specific topic
ros2 topic info /topic_name

# Echo messages from a topic
ros2 topic echo /topic_name

# Check message rate
ros2 topic hz /topic_name

# Publish a test message
ros2 topic pub /topic_name std_msgs/String "data: 'test'"
```

### Monitoring and Diagnostics

```python
from rclpy.qos import qos_profile_sensor_data
from diagnostic_updater import Updater, DiagnosticTask

class MonitoredTopicNode(Node):
    def __init__(self):
        super().__init__('monitored_topic')

        # Setup diagnostics
        self.diag_updater = Updater(self)
        self.diag_updater.add("Topic Status", self.topic_status_diag)

        # Publisher with monitoring
        self.pub = self.create_publisher(String, '/monitored_topic',
                                       qos_profile=qos_profile_sensor_data)

        # Timer for diagnostics
        self.diag_timer = self.create_timer(1.0, self.update_diagnostics)

    def update_diagnostics(self):
        self.diag_updater.update()

    def topic_status_diag(self, stat):
        # Add diagnostic information
        stat.summary(0, "Topic operating normally")
        stat.add("Publisher Count", self.pub.get_subscription_count())
        return stat
```

Understanding and properly implementing topics is fundamental to creating robust, scalable ROS 2 applications. The publish-subscribe pattern enables flexible, decoupled architectures that are essential for complex robotic systems like humanoid robots where multiple subsystems must coordinate seamlessly.
---
sidebar_label: 'Custom Interfaces'
title: 'Custom Interfaces'
---

# Custom Interfaces

## Introduction

Custom interfaces in ROS 2 (Robot Operating System 2) are user-defined message types that enable specialized communication between nodes. While ROS 2 provides a rich set of standard message types in packages like `std_msgs`, `geometry_msgs`, and `sensor_msgs`, custom interfaces allow developers to define domain-specific data structures tailored to their particular robotic applications. Custom interfaces include messages (`.msg`), services (`.srv`), and action (`.action`) definitions that facilitate precise data exchange between nodes in a robot system.

## Understanding Custom Interface Types

### Messages (.msg)

Custom messages define structured data for topic-based communication:

```
# Example: RobotStatus.msg
string robot_name
bool is_operational
float64 battery_level
int32 error_code
geometry_msgs/Pose current_pose
```

Messages support:
- Primitive types: `bool`, `int8`, `uint8`, `float32`, `float64`, `string`, etc.
- Arrays of primitive types: `int32[]`, `string[]`
- Built-in message types: `geometry_msgs/Pose`, `sensor_msgs/JointState`
- Custom message types: `my_robot_msgs/RobotStatus`

### Services (.srv)

Custom services define request-response communication patterns:

```
# Example: SetRobotMode.srv
string mode    # Request: desired robot mode
---
bool success  # Response: operation success
string message # Response: status message
```

### Actions (.action)

Custom actions define goal-result-feedback patterns for long-running operations:

```
# Example: NavigateToPose.action
# Goal
geometry_msgs/PoseStamped target_pose

# Result
bool success
string message

# Feedback
geometry_msgs/Pose current_pose
float32 distance_remaining
```

## Creating Custom Message Interfaces

### Setting Up the Package Structure

To create custom interfaces, establish the proper package structure:

```
my_robot_msgs/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── RobotStatus.msg
│   ├── JointCommand.msg
│   └── SensorArray.msg
├── srv/
│   ├── SetRobotMode.srv
│   └── ExecuteTrajectory.srv
└── action/
    ├── NavigateToPose.action
    └── PickAndPlace.action
```

### Creating Message Definitions

1. Create a `msg/` directory in your package
2. Define your message files with `.msg` extension
3. Use proper syntax with field type and field name

Example custom message (`msg/JointCommand.msg`):
```
# Joint command message for robot control
string joint_name
float64 position
float64 velocity
float64 effort
float64[] trajectory_points  # Array of trajectory points
```

### Creating Service Definitions

1. Create a `srv/` directory in your package
2. Define service files with `.srv` extension
3. Separate request and response with `---`

Example custom service (`srv/ExecuteTrajectory.srv`):
```
# Request: trajectory to execute
trajectory_msgs/JointTrajectory trajectory
bool blocking_execution    # Whether to block until completion
---
# Response: execution result
bool success
string error_message
float64 execution_time
```

### Creating Action Definitions

1. Create an `action/` directory in your package
2. Define action files with `.action` extension
3. Separate goal, result, and feedback with `---`

Example custom action (`action/PickAndPlace.action`):
```
# Goal: pick and place operation
geometry_msgs/PoseStamped object_pose
geometry_msgs/PoseStamped target_pose
string gripper_id
---
# Result: operation result
bool success
string error_message
---
# Feedback: ongoing status
string current_phase  # "approaching", "grasping", "moving", "placing"
float32 progress      # 0.0 to 1.0
geometry_msgs/Pose current_pose
```

## Package Configuration

### CMakeLists.txt Configuration

Configure your `CMakeLists.txt` to build custom interfaces:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Define message files
set(msg_files
  "msg/RobotStatus.msg"
  "msg/JointCommand.msg"
  "msg/SensorArray.msg"
)

# Define service files
set(srv_files
  "srv/SetRobotMode.srv"
  "srv/ExecuteTrajectory.srv"
)

# Define action files
set(action_files
  "action/NavigateToPose.action"
  "action/PickAndPlace.action"
)

# Generate interface files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs trajectory_msgs
)

ament_package()
```

### package.xml Configuration

Update `package.xml` with proper dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_msgs</name>
  <version>0.0.0</version>
  <description>Custom message definitions for my robot</description>
  <maintainer email="developer@example.com">Developer Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>trajectory_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Implementation Examples

### Using Custom Messages in C++

Publisher implementation:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "my_robot_msgs/msg/robot_status.hpp"

class RobotStatusPublisher : public rclcpp::Node
{
public:
  RobotStatusPublisher()
  : Node("robot_status_publisher")
  {
    publisher_ = this->create_publisher<my_robot_msgs::msg::RobotStatus>(
      "robot_status", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RobotStatusPublisher::publish_status, this));
  }

private:
  void publish_status()
  {
    auto msg = my_robot_msgs::msg::RobotStatus();
    msg.robot_name = "my_robot";
    msg.is_operational = true;
    msg.battery_level = 85.5;
    msg.error_code = 0;

    publisher_->publish(msg);
  }

  rclcpp::Publisher<my_robot_msgs::msg::RobotStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

Subscriber implementation:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "my_robot_msgs/msg/robot_status.hpp"

class RobotStatusSubscriber : public rclcpp::Node
{
public:
  RobotStatusSubscriber()
  : Node("robot_status_subscriber")
  {
    subscription_ = this->create_subscription<my_robot_msgs::msg::RobotStatus>(
      "robot_status",
      10,
      [this](const my_robot_msgs::msg::RobotStatus::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(),
          "Robot: %s, Battery: %.2f%%, Operational: %s",
          msg->robot_name.c_str(),
          msg->battery_level,
          msg->is_operational ? "Yes" : "No");
      });
  }

private:
  rclcpp::Subscription<my_robot_msgs::msg::RobotStatus>::SharedPtr subscription_;
};
```

### Using Custom Messages in Python

Publisher implementation:
```python
import rclpy
from rclpy.node import Node
from my_robot_msgs.msg import RobotStatus

class RobotStatusPublisher(Node):

    def __init__(self):
        super().__init__('robot_status_publisher')
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_status)

    def publish_status(self):
        msg = RobotStatus()
        msg.robot_name = 'my_robot'
        msg.is_operational = True
        msg.battery_level = 85.5
        msg.error_code = 0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published status for {msg.robot_name}')
```

Subscriber implementation:
```python
import rclpy
from rclpy.node import Node
from my_robot_msgs.msg import RobotStatus

class RobotStatusSubscriber(Node):

    def __init__(self):
        super().__init__('robot_status_subscriber')
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_callback,
            10)
        self.subscription  # prevent unused variable warning

    def status_callback(self, msg):
        self.get_logger().info(
            f'Robot: {msg.robot_name}, Battery: {msg.battery_level}%, '
            f'Operational: {msg.is_operational}')
```

### Using Custom Services

Service server implementation (C++):
```cpp
#include "rclcpp/rclcpp.hpp"
#include "my_robot_msgs/srv/set_robot_mode.hpp"

class SetRobotModeService : public rclcpp::Node
{
public:
  SetRobotModeService()
  : Node("set_robot_mode_service")
  {
    service_ = this->create_service<my_robot_msgs::srv::SetRobotMode>(
      "set_robot_mode",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<my_robot_msgs::srv::SetRobotMode::Request> request,
             const std::shared_ptr<my_robot_msgs::srv::SetRobotMode::Response> response) {
        (void)request_header;
        // Process the mode change request
        if (request->mode == "autonomous" || request->mode == "manual" || request->mode == "maintenance") {
          response->success = true;
          response->message = "Mode changed to " + request->mode;
        } else {
          response->success = false;
          response->message = "Invalid mode: " + request->mode;
        }
      });
  }

private:
  rclcpp::Service<my_robot_msgs::srv::SetRobotMode>::SharedPtr service_;
};
```

Service client implementation (Python):
```python
import rclpy
from rclpy.node import Node
from my_robot_msgs.srv import SetRobotMode

class SetRobotModeClient(Node):

    def __init__(self):
        super().__init__('set_robot_mode_client')
        self.cli = self.create_client(SetRobotMode, 'set_robot_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, mode):
        request = SetRobotMode.Request()
        request.mode = mode
        future = self.cli.call_async(request)
        return future
```

## Best Practices for Custom Interfaces

### Message Design Guidelines

1. **Use descriptive field names**: Choose clear, unambiguous names for message fields
2. **Follow naming conventions**: Use snake_case for field names
3. **Include units in comments**: Document units where applicable
4. **Define appropriate data types**: Use the most efficient type for your data
5. **Consider message size**: Large messages can impact performance
6. **Version compatibility**: Design for future extensions

### Interface Versioning

Plan for interface evolution:
```
# Version 1.0
string name
int32 id

# Version 2.0 (maintaining backward compatibility)
string name
int32 id
float64 timestamp  # Added new field
string status      # Added new field
```

### Performance Considerations

1. **Minimize message size**: Use appropriate data types and avoid unnecessary fields
2. **Consider serialization overhead**: Complex nested messages have higher overhead
3. **Use arrays judiciously**: Large arrays can impact performance
4. **Implement proper QoS**: Choose appropriate reliability and durability settings

## Advanced Interface Features

### Nested Message Types

Custom messages can include other custom messages:

```
# RobotSystem.msg
my_robot_msgs/RobotStatus status
my_robot_msgs/JointCommand[] joint_commands
sensor_msgs/BatteryState battery
geometry_msgs/Twist velocity
```

### Constant Definitions

Include constants in message definitions:

```
# RobotMode.msg
string mode

# Constants
string AUTONOMOUS = "autonomous"
string MANUAL = "manual"
string MAINTENANCE = "maintenance"
```

### Array and Variable-Length Fields

Use arrays for variable-length data:

```
# SensorArray.msg
string sensor_type
float64[] readings          # Variable number of sensor readings
string[] sensor_names      # Variable number of sensor names
```

## Interface Testing and Validation

### Testing Custom Interfaces

Create unit tests for interface usage:

```cpp
#include "gtest/gtest.h"
#include "my_robot_msgs/msg/robot_status.hpp"

TEST(RobotStatusTest, DefaultValues) {
  auto msg = my_robot_msgs::msg::RobotStatus();
  EXPECT_EQ(msg.robot_name, "");
  EXPECT_EQ(msg.is_operational, false);
  EXPECT_EQ(msg.battery_level, 0.0);
  EXPECT_EQ(msg.error_code, 0);
}
```

### Interface Validation Tools

Use ROS 2 tools for interface validation:

- `ros2 interface show <interface_type>`: Display interface definition
- `ros2 topic echo <topic_name>`: Monitor topic messages
- `ros2 service call <service_name> <service_type> <args>`: Test service calls

## Integration with Robot Systems

### Navigation Stack Integration

Custom interfaces for navigation:
```
# NavigationGoal.msg
geometry_msgs/PoseStamped target_pose
string navigation_mode  # "global", "local", "emergency_stop"
float64 timeout
```

### Manipulation Stack Integration

Custom interfaces for manipulation:
```
# GraspRequest.msg
geometry_msgs/PoseStamped object_pose
string gripper_id
float64 grasp_force
bool execute_grasp
```

### Sensor Integration

Custom interfaces for sensor data:
```
# MultiSensorData.msg
sensor_msgs/Imu imu_data
sensor_msgs/LaserScan laser_scan
sensor_msgs/Image[] camera_images
string[] sensor_ids
builtin_interfaces/Time timestamp
```

## Troubleshooting Common Issues

### Build Issues

Common CMakeLists.txt errors:
- Missing `rosidl_default_generators` dependency
- Incorrect file paths in `rosidl_generate_interfaces`
- Missing interface dependencies

### Runtime Issues

Common runtime problems:
- Interface type mismatch between nodes
- Package not found during runtime
- Serialization/deserialization errors

### Debugging Strategies

1. Use `ros2 interface list` to verify interface availability
2. Check package installation with `ament list_packages`
3. Verify message definitions with `ros2 interface show`
4. Monitor topics with `ros2 topic echo` to validate message structure

## Conclusion

Custom interfaces are fundamental to building specialized robotic applications in ROS 2. They enable precise data exchange tailored to specific robot capabilities and requirements. Proper design and implementation of custom interfaces enhance system modularity, maintainability, and performance. Following best practices for interface design, versioning, and testing ensures robust and scalable robot systems that can evolve with changing requirements.
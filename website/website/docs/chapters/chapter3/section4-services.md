---
sidebar_label: 'Services'
title: 'Services'
---

# Services

## Introduction

Services in ROS 2 (Robot Operating System 2) represent a fundamental communication paradigm that enables request-response interactions between nodes. Unlike topics, which provide asynchronous, many-to-many communication, services establish synchronous, one-to-one communication where a client sends a request and receives a response from a service server. This communication pattern is essential for operations that require guaranteed delivery, state queries, and command execution with immediate feedback.

## Understanding ROS 2 Services

### Service Architecture

ROS 2 services operate on a client-server model where:

- **Service Server**: A node that provides a specific functionality and waits for requests
- **Service Client**: A node that requests the service functionality and waits for a response
- **Service Interface**: Defines the request and response message types in `.srv` files

The communication follows a synchronous pattern where the client blocks until it receives a response from the server, making it suitable for operations that require immediate results.

### Service vs. Topic Communication

| Aspect | Topics | Services |
|--------|--------|----------|
| Communication Pattern | Publish/Subscribe (asynchronous) | Request/Response (synchronous) |
| Data Flow | One-way (publisher to subscriber) | Two-way (request and response) |
| Delivery Guarantee | Best-effort | Guaranteed delivery |
| Blocking | Non-blocking | Blocking until response |
| Use Cases | Continuous data streams | State queries, commands, calculations |

## Service Definition and Structure

### Service Message Format

ROS 2 service definitions are stored in `.srv` files with the following structure:

```
# Request message
string command
int32 value
---
# Response message
bool success
string message
int32 result
```

The `---` separator divides the request and response message definitions. Each section can contain multiple fields of different data types.

### Common Service Types

ROS 2 provides several built-in service types:

- `std_srvs/Empty`: Simple service with no request or response data
- `std_srvs/SetBool`: Service for setting boolean values
- `std_srvs/Trigger`: Service for triggering actions with success status
- `std_srvs/SetString`: Service for setting string values
- `rcl_interfaces/SetParameters`: Service for setting node parameters

## Service Implementation in ROS 2

### Creating Custom Service Definitions

To create a custom service:

1. Create a `.srv` file in the `srv/` directory of your package
2. Define request and response message structures
3. Add the service to `CMakeLists.txt` and `package.xml`
4. Build the package to generate language-specific interfaces

Example custom service definition (`AddTwoInts.srv`):
```
int64 a
int64 b
---
int64 sum
```

### Implementing a Service Server

In C++:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MinimalService : public rclcpp::Node
{
public:
  MinimalService()
  : Node("minimal_service")
  {
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
             const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
        (void)request_header;
        response->sum = request->a + request->b;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: %ld + %ld = %ld",
                    request->a, request->b, response->sum);
      });
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};
```

In Python:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request: %d + %d = %d' % (request.a, request.b, response.sum))
        return response
```

### Implementing a Service Client

In C++:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MinimalClient : public rclcpp::Node
{
public:
  MinimalClient()
  : Node("minimal_client")
  {
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }
  }

  void send_request()
  {
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 2;
    request->b = 3;

    auto result_future = client_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);
    auto result = result_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result of add_two_ints: %ld", result->sum);
  }

private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};
```

In Python:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

## Advanced Service Features

### Service Quality of Service (QoS)

ROS 2 services support QoS settings for reliability and durability:

```cpp
// Set QoS for service
auto service_qos = rclcpp::QoS(10);
service_qos.reliable();  // Ensure reliable delivery
service_qos.durability_volatile();  // Volatile durability
```

### Service Callback Groups

For complex nodes with multiple services, callback groups allow for parallel execution:

```cpp
// Create callback group
auto callback_group = this->create_callback_group(
  rclcpp::CallbackGroupType::MutuallyExclusive);

// Assign service to callback group
rclcpp::SubscriptionOptions options;
options.callback_group = callback_group;
```

### Service Discovery and Introspection

ROS 2 provides tools for discovering available services:

- `ros2 service list`: List all available services
- `ros2 service info <service_name>`: Get detailed information about a service
- `ros2 service type <service_name>`: Get the service type
- `ros2 service call <service_name> <service_type> <args>`: Call a service directly

## Service Patterns in Robotics

### State Query Services

Services are ideal for querying robot state:

```cpp
// Example: Get robot position
service: /get_robot_position
request: (empty)
response: geometry_msgs::msg::Pose pose
```

### Configuration Services

Services for changing robot configuration:

```cpp
// Example: Set robot parameters
service: /set_robot_mode
request: string mode
response: bool success, string message
```

### Action Services

For operations that need immediate results:

```cpp
// Example: Trigger robot action
service: /trigger_action
request: string action_name
response: bool success, string result
```

## Error Handling and Robustness

### Service Availability Checking

Always check if services are available before calling:

```cpp
// Wait for service with timeout
if (!client_->wait_for_service(std::chrono::seconds(5))) {
  RCLCPP_ERROR(this->get_logger(), "Service not available");
  return;
}
```

### Timeout Handling

Implement timeouts to prevent indefinite blocking:

```cpp
auto result_future = client_->async_send_request(request);
auto future_status = rclcpp::spin_until_future_complete(
  this->get_node_base_interface(), result_future, std::chrono::seconds(5));

if (future_status == rclcpp::FutureReturnCode::SUCCESS) {
  // Process result
} else {
  RCLCPP_ERROR(this->get_logger(), "Service call failed");
}
```

### Exception Handling

Handle potential service call exceptions:

```cpp
try {
  auto result = result_future.get();
  // Process result
} catch (const std::exception& e) {
  RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
}
```

## Performance Considerations

### Synchronous vs. Asynchronous Calls

For non-blocking operations, use asynchronous service calls:

```cpp
// Asynchronous call
auto future_result = client_->async_send_request(request);
// Continue with other operations
// Check result later or use callbacks
```

### Service Threading

Consider threading implications when designing services:

- Service callbacks run in the executor's thread pool
- Avoid blocking operations in service callbacks
- Use separate threads for long-running operations

### Resource Management

Properly manage service resources:

```cpp
// Clean up services when node is destroyed
// Services are automatically cleaned up with the node
```

## Best Practices

### Service Design Guidelines

1. **Use services for request-response patterns**: When you need guaranteed delivery and immediate response
2. **Keep services lightweight**: Avoid heavy computations in service callbacks
3. **Implement proper error handling**: Return appropriate error codes in responses
4. **Use descriptive names**: Service names should clearly indicate their function
5. **Document service interfaces**: Clearly specify request/response fields and expected behavior

### Security Considerations

- Validate input parameters in service callbacks
- Implement authentication for critical services
- Use secure communication channels when needed
- Limit service access to authorized nodes

## Integration with Robot Systems

### Service in Navigation Stack

Navigation systems use services for configuration and control:

```cpp
// Set new goal
service: /navigate_to_pose
request: geometry_msgs::msg::PoseStamped goal
response: bool accepted
```

### Service in Manipulation Stack

Manipulation systems use services for grasp planning and execution:

```cpp
// Plan grasp
service: /plan_grasp
request: geometry_msgs::msg::Pose object_pose
response: moveit_msgs::msg::MotionPlanResponse plan
```

## Conclusion

ROS 2 services provide a crucial synchronous communication mechanism that complements the asynchronous topic-based communication. They are essential for operations requiring guaranteed delivery, state queries, and immediate responses. Understanding service implementation, usage patterns, and best practices is fundamental for building robust robotic systems with proper client-server interactions. Proper use of services enhances the reliability and maintainability of robot software architectures.
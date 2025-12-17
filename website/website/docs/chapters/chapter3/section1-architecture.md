---
sidebar_position: 1
title: "Architecture"
---

# Architecture

## Introduction

The architecture of ROS 2 (Robot Operating System 2) represents a fundamental shift from its predecessor, designed specifically to address the needs of modern robotics applications including humanoid robots, autonomous systems, and industrial automation. ROS 2's architecture is built on modern distributed computing principles, providing improved real-time capabilities, enhanced security, and better support for commercial deployment compared to ROS 1. Understanding ROS 2's architecture is crucial for developing robust, scalable, and maintainable robotic systems.

## Evolution from ROS 1 to ROS 2

### Limitations of ROS 1 Architecture

ROS 1, while revolutionary for robotics research, had several architectural limitations:

- **Single Master**: Centralized master node created a single point of failure
- **No Real-time Support**: Not designed for real-time or safety-critical applications
- **Security Concerns**: No built-in security mechanisms
- **Limited Multi-robot Support**: Challenging to coordinate multiple robots
- **Distribution Challenges**: Difficult to deploy in commercial environments

### ROS 2 Design Philosophy

ROS 2 was architected to address these limitations:

- **Distributed Architecture**: No single point of failure
- **Real-time Ready**: Designed with real-time systems in mind
- **Security First**: Built-in security and authentication
- **Commercial Deployment**: Ready for production environments
- **Standards Compliant**: Based on industry standards like DDS

## Core Architecture Components

### Data Distribution Service (DDS)

DDS (Data Distribution Service) forms the foundation of ROS 2's communication architecture:

#### DDS Fundamentals
- **Publish-Subscribe Model**: Decentralized communication pattern
- **Data-Centric**: Focus on data rather than communication endpoints
- **Quality of Service (QoS)**: Configurable communication policies
- **Platform Independence**: Language and platform agnostic

#### DDS Implementation in ROS 2
- **Middleware Abstraction**: Pluggable DDS implementations (Fast DDS, Cyclone DDS, RTI Connext)
- **Discovery Protocol**: Automatic discovery of nodes and topics
- **Reliable Communication**: Guaranteed message delivery options
- **Real-time Capabilities**: Support for real-time communication requirements

### Nodes and Processes

#### Node Architecture
- **Process Isolation**: Each node runs in its own process
- **Memory Protection**: Process-level memory isolation
- **Crash Containment**: Node crashes don't affect other nodes
- **Resource Management**: Independent resource allocation per node

#### Node Communication
- **Intra-process Communication**: Optimized communication within the same process
- **Inter-process Communication**: Standard communication between processes
- **Network Communication**: Seamless communication across network boundaries
- **Transport Layer Abstraction**: Multiple transport options (TCP, UDP, shared memory)

## Communication Patterns

### Topics (Publish-Subscribe)

#### Topic Architecture
- **Anonymous Communication**: Publishers and subscribers don't know each other
- **Loose Coupling**: Decoupling of communication endpoints
- **Data Flow**: Unidirectional data flow from publishers to subscribers
- **Message Types**: Strongly typed messages with automatic serialization

#### Quality of Service (QoS) Settings
- **Reliability**: Reliable vs. best-effort delivery
- **Durability**: Volatile vs. transient-local durability
- **History**: Keep-all vs. keep-last history policies
- **Deadline**: Time bounds for message delivery
- **Liveliness**: Detection of active publishers/subscribers

### Services (Request-Response)

#### Service Architecture
- **Synchronous Communication**: Request followed by response pattern
- **Bidirectional Data Flow**: Request and response messages
- **Blocking Calls**: Synchronous service calls block execution
- **Asynchronous Options**: Non-blocking service call alternatives

#### Service Characteristics
- **One-to-One**: Single client communicates with single server
- **Stateful Operations**: Services can maintain state between calls
- **Error Handling**: Structured error reporting and handling
- **Timeout Management**: Configurable timeout for service calls

### Actions (Goal-Feedback-Result)

#### Action Architecture
- **Long-running Operations**: Designed for extended operations
- **Three-part Communication**: Goal, feedback, and result messages
- **Preemption Support**: Ability to cancel ongoing operations
- **Status Tracking**: Continuous operation status updates

#### Action Components
- **Action Client**: Initiates and monitors action goals
- **Action Server**: Executes action goals and provides feedback
- **Goal Handling**: Manages goal acceptance and execution
- **Feedback Loop**: Continuous feedback during operation

## Client Library Architecture

### rclcpp and rclpy

#### Common Client Library (rcl)
- **C-based Foundation**: Low-level C library providing common functionality
- **Language Bindings**: Shared implementation across language bindings
- **DDS Abstraction**: Abstracts DDS middleware differences
- **ROS 2 Abstraction**: Provides ROS 2-specific functionality

#### C++ Client Library (rclcpp)
- **Performance Optimized**: Designed for high-performance applications
- **Memory Management**: Efficient memory allocation and management
- **Template-Based**: Extensive use of C++ templates for type safety
- **Real-time Considerations**: Designed with real-time systems in mind

#### Python Client Library (rclpy)
- **Ease of Use**: Python-friendly interface for rapid development
- **Dynamic Typing**: Leverages Python's dynamic nature
- **Integration**: Easy integration with Python-based tools
- **Prototyping**: Ideal for rapid prototyping and experimentation

## Parameter System

### Parameter Architecture
- **Hierarchical Parameters**: Organized in a tree-like structure
- **Node-specific Parameters**: Parameters scoped to individual nodes
- **Dynamic Reconfiguration**: Runtime parameter modification
- **Type Safety**: Strong typing with validation

### Parameter Management
- **Declarative Parameters**: Nodes can declare expected parameters
- **Parameter Descriptors**: Metadata about parameter constraints
- **Validation**: Automatic validation of parameter values
- **Persistence**: Saving and loading parameter configurations

## Launch System

### Launch Architecture
- **Declarative Launch**: XML and YAML-based launch configuration
- **Python Launch**: Programmatic launch file creation
- **Composable Nodes**: Running multiple nodes in single process
- **Lifecycle Management**: Managing node lifecycle states

### Launch Features
- **Conditional Launch**: Launch nodes based on conditions
- **Arguments and Substitution**: Parameterized launch files
- **Event Handling**: Reacting to node lifecycle events
- **Monitoring**: Launch file execution monitoring

## Security Architecture

### Security by Design
- **Authentication**: Identity verification for nodes and users
- **Authorization**: Permission-based access control
- **Encryption**: Data encryption in transit and at rest
- **Audit Trail**: Logging security-relevant events

### Security Implementation
- **DDS Security**: Leverages DDS security standard
- **Certificate Management**: PKI-based certificate system
- **Access Control Lists**: Fine-grained access control
- **Secure Discovery**: Secured node and topic discovery

## Real-time Considerations

### Real-time Architecture Features
- **Deterministic Communication**: Predictable communication timing
- **Memory Allocation**: Avoiding dynamic allocation in critical paths
- **Thread Management**: Configurable threading models
- **Priority Control**: Thread priority assignment and control

### Real-time Optimizations
- **Lock-free Data Structures**: Reducing contention in critical paths
- **Zero-copy Communication**: Minimizing memory copies
- **Deadline Scheduling**: Support for deadline-based scheduling
- **Cache Optimization**: Memory access pattern optimization

## Middleware Abstraction Layer (RMW)

### RMW Architecture
- **DDS Abstraction**: Abstracts different DDS implementations
- **Plug-in Architecture**: Support for multiple DDS vendors
- **API Consistency**: Consistent API across different DDS implementations
- **Performance Tuning**: DDS-specific optimizations

### Supported DDS Implementations
- **Fast DDS**: eProsima's high-performance DDS implementation
- **Cyclone DDS**: Eclipse Foundation's DDS implementation
- **RTI Connext DDS**: Commercial DDS implementation
- **OpenDDS**: Open-source DDS implementation

## Build System Architecture

### colcon Build System
- **Multi-package Builds**: Building multiple packages efficiently
- **Dependency Management**: Automatic dependency resolution
- **Build Isolation**: Isolated builds to prevent conflicts
- **Extensible**: Plugin-based architecture for different build types

### Package Management
- **ament**: ROS 2's package build and runtime system
- **CMake Integration**: Seamless integration with CMake
- **Python Packages**: Support for Python-only packages
- **Cross-compilation**: Support for cross-platform builds

## Network Architecture

### Communication Topology
- **Peer-to-Peer**: Direct communication between nodes
- **Multi-cast Support**: Efficient multi-cast communication
- **Unicast Communication**: Point-to-point communication
- **Network Discovery**: Automatic network topology discovery

### Network Configuration
- **ROS_DOMAIN_ID**: Isolating communication domains
- **Network Partitions**: Configuring network segments
- **Bandwidth Management**: Controlling network resource usage
- **Quality of Service**: Network-level QoS configuration

## Lifecycle Node Architecture

### Node Lifecycle States
- **Unconfigured**: Node created but not configured
- **Inactive**: Configured but not active
- **Active**: Fully operational state
- **Finalized**: Node has been shut down

### Lifecycle Management Benefits
- **Orderly Transitions**: Controlled state transitions
- **Resource Management**: Proper resource allocation/deallocation
- **Error Recovery**: Graceful handling of errors
- **Supervision**: External lifecycle management

## Integration Architecture

### Third-party Integration
- **Existing Libraries**: Integration with non-ROS libraries
- **Legacy Systems**: Connecting to existing systems
- **Commercial Software**: Integration with commercial tools
- **Hardware Drivers**: Connecting to hardware interfaces

### External System Integration
- **Cloud Services**: Connecting to cloud-based services
- **Web Services**: Integration with REST and other web APIs
- **Databases**: Integration with data storage systems
- **Enterprise Systems**: Integration with business systems

## Performance Architecture

### Performance Monitoring
- **Node Statistics**: Monitoring node performance metrics
- **Communication Latency**: Measuring message delivery times
- **CPU and Memory**: Resource usage monitoring
- **Network Performance**: Network communication metrics

### Performance Optimization
- **Communication Patterns**: Choosing optimal communication patterns
- **Message Design**: Optimizing message structures
- **Threading Models**: Configuring optimal threading
- **Resource Allocation**: Efficient resource management

## Architecture Best Practices

### Design Principles
- **Modularity**: Designing modular, reusable components
- **Loose Coupling**: Minimizing dependencies between components
- **High Cohesion**: Grouping related functionality
- **Separation of Concerns**: Separating different aspects of functionality

### Implementation Guidelines
- **Error Handling**: Robust error handling and recovery
- **Resource Management**: Proper resource allocation and cleanup
- **Testing**: Comprehensive testing of architectural components
- **Documentation**: Clear documentation of architectural decisions

## Future Architecture Trends

### Emerging Technologies
- **Edge Computing**: Integration with edge computing platforms
- **5G Integration**: Leveraging 5G for robot communication
- **AI Framework Integration**: Better integration with AI frameworks
- **Quantum Computing**: Potential future integration points

### Architecture Evolution
- **Microservices**: Adapting microservices patterns to robotics
- **Cloud Robotics**: Enhanced cloud integration capabilities
- **Federated Learning**: Distributed learning architectures
- **Digital Twins**: Integration with digital twin technologies

## Conclusion

ROS 2's architecture represents a significant advancement over ROS 1, addressing critical needs for modern robotics applications including humanoid robots. Its distributed, secure, and real-time capable design makes it suitable for both research and commercial applications. The architecture's flexibility allows for various deployment scenarios while maintaining the modularity and reusability that made ROS popular. As robotics continues to evolve, ROS 2's architecture provides a solid foundation for building increasingly sophisticated and capable robotic systems.
---
sidebar_label: 'Forward & Inverse Kinematics'
title: 'Forward & Inverse Kinematics'
---

# Forward & Inverse Kinematics

## Introduction

Forward Kinematics (FK) and Inverse Kinematics (IK) are fundamental concepts in robotics that describe the relationship between joint angles and end-effector position and orientation. Forward Kinematics calculates the end-effector pose given joint angles, while Inverse Kinematics determines the required joint angles to achieve a desired end-effector pose. These mathematical frameworks are essential for robot control, motion planning, manipulation, and trajectory generation in robotic systems.

## Forward Kinematics

### Definition and Purpose

Forward Kinematics is the process of calculating the position and orientation of a robot's end-effector based on the known joint angles and link parameters. It answers the question: "Given the joint angles, where is the end-effector?"

Forward kinematics is deterministic - for a given set of joint angles, there is exactly one resulting end-effector pose (assuming no kinematic redundancy).

### Mathematical Foundation

Forward kinematics relies on transformation matrices to represent the relationship between coordinate frames attached to each link of the robot.

For a serial manipulator with n joints, the end-effector transformation is:
```
T_0^n = T_0^1 * T_1^2 * T_2^3 * ... * T_(n-1)^n
```

Where T_i^(i+1) represents the transformation from frame i to frame i+1.

### Denavit-Hartenberg (DH) Parameters

The DH convention provides a systematic method for defining coordinate frames on robot links:

- **a_i**: Link length (distance along x_i from z_i to z_(i+1))
- **α_i**: Link twist (angle from z_i to z_(i+1) about x_i)
- **d_i**: Link offset (distance along z_i from x_i to x_(i+1))
- **θ_i**: Joint angle (angle from x_i to x_(i+1) about z_i)

### Transformation Matrix

Using DH parameters, the transformation matrix from frame i to frame i+1 is:

```
     | cos(θ_i)   -sin(θ_i)cos(α_i)   sin(θ_i)sin(α_i)   a_i*cos(θ_i) |
T =  | sin(θ_i)    cos(θ_i)cos(α_i)  -cos(θ_i)sin(α_i)   a_i*sin(θ_i) |
     | 0           sin(α_i)           cos(α_i)            d_i         |
     | 0           0                  0                   1           |
```

### Example: 2-DOF Planar Manipulator

Consider a simple 2-DOF planar manipulator with link lengths L1 and L2:

Forward kinematics equations:
```
x = L1*cos(θ1) + L2*cos(θ1 + θ2)
y = L1*sin(θ1) + L2*sin(θ1 + θ2)
```

Where θ1 and θ2 are the joint angles.

### Implementation in Code

C++ implementation example:
```cpp
#include <cmath>
#include <vector>

struct Pose {
    double x, y, z;
    double roll, pitch, yaw;
};

class ForwardKinematics {
public:
    ForwardKinematics(const std::vector<double>& link_lengths)
        : link_lengths_(link_lengths) {}

    Pose calculate_pose(const std::vector<double>& joint_angles) {
        double x = 0.0, y = 0.0;
        double cumulative_angle = 0.0;

        for (size_t i = 0; i < joint_angles.size(); ++i) {
            cumulative_angle += joint_angles[i];
            x += link_lengths_[i] * cos(cumulative_angle);
            y += link_lengths_[i] * sin(cumulative_angle);
        }

        Pose result;
        result.x = x;
        result.y = y;
        result.z = 0.0;
        result.roll = 0.0;
        result.pitch = 0.0;
        result.yaw = cumulative_angle;

        return result;
    }

private:
    std::vector<double> link_lengths_;
};
```

Python implementation example:
```python
import numpy as np

def forward_kinematics_2dof(joint_angles, link_lengths):
    """
    Calculate forward kinematics for a 2-DOF planar manipulator
    """
    theta1, theta2 = joint_angles
    L1, L2 = link_lengths

    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)

    return np.array([x, y, 0.0])  # Return position as [x, y, z]

def dh_transform(a, alpha, d, theta):
    """
    Calculate DH transformation matrix
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])
```

## Inverse Kinematics

### Definition and Purpose

Inverse Kinematics is the process of determining the joint angles required to achieve a desired end-effector position and orientation. It answers the question: "Where should the joints be to place the end-effector at a specific location?"

Inverse kinematics is generally more complex than forward kinematics and may have multiple solutions, no solutions, or infinite solutions (for redundant manipulators).

### Mathematical Challenges

Inverse kinematics presents several challenges:

- **Multiple solutions**: A given end-effector pose may be achievable through multiple joint configurations
- **No solutions**: Desired pose may be outside the robot's workspace
- **Singularities**: Mathematical conditions where the Jacobian matrix becomes singular
- **Computational complexity**: Analytical solutions are only possible for simple robots

### Analytical vs. Numerical Methods

#### Analytical Methods
- Provide exact, closed-form solutions
- Fast computation
- Limited to simple robot configurations
- Require manual derivation for each robot

#### Numerical Methods
- General-purpose approaches
- Can handle complex robots
- Iterative solutions
- May not converge or converge to local minima

### Common Numerical Methods

#### Jacobian-Based Methods

The Jacobian matrix relates joint velocities to end-effector velocities:
```
v = J(θ) * θ̇
```

Where:
- v is the end-effector velocity vector
- J(θ) is the Jacobian matrix
- θ̇ is the joint velocity vector

**Jacobian Transpose Method:**
```
θ̇ = J^T * v
```

**Pseudoinverse Method:**
```
θ̇ = J^+ * v = (J^T * J)^(-1) * J^T * v
```

#### Iterative Methods

**Cyclic Coordinate Descent (CCD):**
- Adjusts one joint at a time
- Simple and robust
- Good for reaching targets
- May not find optimal solutions

**Damped Least Squares:**
- More stable than pseudoinverse
- Handles singularities better
- Parameter to balance accuracy and stability

### Implementation Examples

C++ implementation using Jacobian:
```cpp
#include <Eigen/Dense>
#include <vector>

class InverseKinematics {
public:
    InverseKinematics(const std::vector<double>& link_lengths)
        : link_lengths_(link_lengths) {}

    std::vector<double> solve_ik(const Eigen::Vector3d& target_pose,
                                std::vector<double> initial_joints,
                                double tolerance = 0.001,
                                int max_iterations = 100) {

        std::vector<double> joints = initial_joints;

        for (int iter = 0; iter < max_iterations; ++iter) {
            // Calculate current end-effector position
            auto current_pose = calculate_forward_kinematics(joints);

            // Calculate error
            Eigen::Vector3d error = target_pose - current_pose;

            if (error.norm() < tolerance) {
                break;  // Converged
            }

            // Calculate Jacobian
            Eigen::MatrixXd jacobian = calculate_jacobian(joints);

            // Update joints using pseudoinverse
            Eigen::VectorXd joint_delta = jacobian.completeOrthogonalDecomposition().pseudoInverse() * error;

            // Apply joint updates
            for (size_t i = 0; i < joints.size(); ++i) {
                joints[i] += joint_delta[i];
            }
        }

        return joints;
    }

private:
    std::vector<double> link_lengths_;

    Eigen::Vector3d calculate_forward_kinematics(const std::vector<double>& joints) {
        // Implementation of forward kinematics for position calculation
        double x = 0.0, y = 0.0;
        double cumulative_angle = 0.0;

        for (size_t i = 0; i < joints.size(); ++i) {
            cumulative_angle += joints[i];
            x += link_lengths_[i] * cos(cumulative_angle);
            y += link_lengths_[i] * sin(cumulative_angle);
        }

        return Eigen::Vector3d(x, y, 0.0);
    }

    Eigen::MatrixXd calculate_jacobian(const std::vector<double>& joints) {
        size_t n = joints.size();
        Eigen::MatrixXd jacobian(3, n);  // 3 for x, y, z

        double cumulative_angle = 0.0;
        double x = 0.0, y = 0.0;

        // Calculate cumulative position and angles
        for (size_t i = 0; i < n; ++i) {
            cumulative_angle += joints[i];
            x += link_lengths_[i] * cos(cumulative_angle);
            y += link_lengths_[i] * sin(cumulative_angle);
        }

        cumulative_angle = 0.0;
        double current_x = 0.0, current_y = 0.0;

        for (size_t i = 0; i < n; ++i) {
            cumulative_angle += joints[i];
            current_x += link_lengths_[i] * cos(cumulative_angle);
            current_y += link_lengths_[i] * sin(cumulative_angle);

            // Calculate partial derivatives
            jacobian(0, i) = -(y - current_y);  // dx/dθ_i
            jacobian(1, i) = x - current_x;     // dy/dθ_i
            jacobian(2, i) = 1.0;               // dz/dθ_i (assuming planar motion)
        }

        return jacobian;
    }
};
```

Python implementation using numerical methods:
```python
import numpy as np
from scipy.optimize import minimize

def jacobian_2dof(joint_angles, link_lengths):
    """
    Calculate Jacobian matrix for 2-DOF planar manipulator
    """
    theta1, theta2 = joint_angles
    L1, L2 = link_lengths

    # Partial derivatives of x and y with respect to theta1 and theta2
    J = np.array([
        [-L1*np.sin(theta1) - L2*np.sin(theta1 + theta2), -L2*np.sin(theta1 + theta2)],
        [L1*np.cos(theta1) + L2*np.cos(theta1 + theta2), L2*np.cos(theta1 + theta2)]
    ])

    return J

def inverse_kinematics_jacobian(target_pos, initial_angles, link_lengths,
                               max_iterations=100, tolerance=1e-6):
    """
    Solve inverse kinematics using Jacobian pseudoinverse method
    """
    joint_angles = np.array(initial_angles)

    for i in range(max_iterations):
        # Calculate current position
        current_pos = forward_kinematics_2dof(joint_angles, link_lengths)[:2]

        # Calculate error
        error = target_pos - current_pos

        if np.linalg.norm(error) < tolerance:
            break

        # Calculate Jacobian
        J = jacobian_2dof(joint_angles, link_lengths)

        # Calculate joint angle updates using pseudoinverse
        joint_delta = np.linalg.pinv(J) @ error

        # Update joint angles
        joint_angles += joint_delta

    return joint_angles

def ik_objective_function(joint_angles, target_pos, link_lengths):
    """
    Objective function for optimization-based IK
    """
    current_pos = forward_kinematics_2dof(joint_angles, link_lengths)[:2]
    return np.sum((current_pos - target_pos)**2)

def inverse_kinematics_optimization(target_pos, initial_angles, link_lengths):
    """
    Solve inverse kinematics using optimization
    """
    result = minimize(
        ik_objective_function,
        initial_angles,
        args=(target_pos, link_lengths),
        method='BFGS'
    )

    return result.x
```

## Workspace Analysis

### Reachable Workspace
- The set of all points that the end-effector can reach
- Depends on link lengths and joint limits
- Inner and outer boundaries defined by geometric constraints

### Dexterity Workspace
- Subset of reachable workspace where the manipulator can achieve any orientation
- Important for tasks requiring specific tool orientations

### Joint Limits Considerations
- Joint limits restrict the workspace
- Can cause multiple disconnected workspace regions
- Must be considered in IK solutions

## Singularity Analysis

### Types of Singularities

**Boundary Singularities:**
- Occur when the end-effector is at the workspace boundary
- Loss of motion in one or more directions

**Interior Singularities:**
- Occur within the workspace
- Often caused by aligned joint axes

### Detection and Handling
- Monitor determinant of Jacobian matrix
- Use damped least squares method near singularities
- Implement singularity avoidance strategies

## Applications in Robotics

### Motion Planning
- Trajectory generation in joint space
- Path planning in Cartesian space
- Smooth motion interpolation

### Manipulation Tasks
- Grasp planning and execution
- Tool pose control
- Assembly operations

### Control Systems
- Cartesian position control
- Force control in end-effector frame
- Visual servoing

## ROS Integration

### MoveIt! Integration
MoveIt! provides advanced IK capabilities:
- Multiple IK solvers (KDL, TRAC-IK, analytical solvers)
- Collision-aware IK
- Cartesian path planning

Example MoveIt! IK call:
```cpp
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>

// Initialize kinematics solver
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

// Set joint positions
std::vector<double> joint_values = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
kinematic_state->setJointGroupPositions("manipulator", joint_values);

// Calculate FK
Eigen::Vector3d end_effector_pos;
Eigen::Quaterniond end_effector_rot;
const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");
end_effector_pos = end_effector_state.translation();
end_effector_rot = Eigen::Quaterniond(end_effector_state.rotation());
```

## Advanced Topics

### Redundant Manipulators
- More degrees of freedom than required for task
- Infinite solutions to IK problem
- Additional optimization criteria possible (e.g., obstacle avoidance, joint limit minimization)

### Parallel Manipulators
- Multiple kinematic chains connecting base to end-effector
- More complex FK and IK mathematics
- Examples: Stewart platform, Delta robot

### Humanoid Robot Kinematics
- Complex kinematic chains for arms and legs
- Multiple IK targets (e.g., both hands and head)
- Balance and stability considerations

## Best Practices

### Numerical Stability
- Use appropriate tolerance values
- Handle singularities gracefully
- Monitor solution convergence

### Performance Optimization
- Pre-compute constant terms
- Use efficient matrix operations
- Consider closed-form solutions when available

### Error Handling
- Check for workspace boundaries
- Validate joint limits
- Implement fallback strategies

## Common Pitfalls and Troubleshooting

### Convergence Issues
- Poor initial guess for iterative methods
- Target pose outside workspace
- Ill-conditioned Jacobian matrix

### Multiple Solutions
- Need criteria to select appropriate solution
- Consider joint limit constraints
- Use optimization with additional objectives

### Numerical Accuracy
- Floating-point precision limitations
- Accumulated errors in long kinematic chains
- Proper scaling of variables

## Conclusion

Forward and Inverse Kinematics form the mathematical foundation for robot control and motion planning. Forward kinematics provides a deterministic mapping from joint space to Cartesian space, while inverse kinematics enables the control of robot end-effectors in Cartesian coordinates. Understanding both analytical and numerical methods for solving these problems is essential for developing effective robotic systems. The choice of method depends on the robot's complexity, real-time requirements, and accuracy needs. Modern robotics frameworks like MoveIt! provide sophisticated tools for handling these computations, but understanding the underlying principles remains crucial for effective robot programming and troubleshooting.
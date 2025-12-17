---
sidebar_label: 'DH Parameters'
title: 'DH Parameters'
---

# DH Parameters

## Introduction

The Denavit-Hartenberg (DH) parameters provide a systematic method for defining coordinate frames on robot links and describing the kinematic structure of serial manipulators. Developed by Jacques Denavit and Richard Hartenberg in 1955, the DH convention is a widely adopted standard in robotics for representing the geometry of kinematic chains. This mathematical framework enables the systematic derivation of forward kinematics equations and forms the foundation for robot modeling, analysis, and control.

## DH Parameter Convention

### Overview

The DH convention uses four parameters to define the relationship between consecutive coordinate frames in a kinematic chain:

- **a_i** (link length): Distance along x_i from z_i to z_(i+1)
- **α_i** (link twist): Angle from z_i to z_(i+1) about x_i
- **d_i** (link offset): Distance along z_i from x_i to x_(i+1)
- **θ_i** (joint angle): Angle from x_i to x_(i+1) about z_i

### Frame Assignment Rules

The DH convention follows specific rules for assigning coordinate frames:

1. **z-axis**: Aligned with the joint axis (axis of motion for prismatic joints or axis of rotation for revolute joints)

2. **x-axis**: Points along the common normal from z_i to z_(i+1), or in the direction normal to z_i if the axes intersect

3. **y-axis**: Completes the right-handed coordinate system (y = z × x)

### Standard vs. Modified DH Parameters

Two main DH parameter conventions exist:

**Standard DH Parameters:**
- Frame i is attached to link i
- Transformation is: Rot(z, θ_i) × Trans(z, d_i) × Trans(x, a_i) × Rot(x, α_i)

**Modified DH Parameters:**
- Frame i is attached to link i-1
- Transformation is: Rot(x, α_i) × Trans(x, a_i) × Rot(z, θ_i) × Trans(z, d_i)

## Mathematical Formulation

### Transformation Matrix

The transformation matrix from frame i to frame i+1 using standard DH parameters is:

```
     | cos(θ_i)   -sin(θ_i)cos(α_i)   sin(θ_i)sin(α_i)   a_i*cos(θ_i) |
T =  | sin(θ_i)    cos(θ_i)cos(α_i)  -cos(θ_i)sin(α_i)   a_i*sin(θ_i) |
     | 0           sin(α_i)           cos(α_i)            d_i         |
     | 0           0                  0                   1           |
```

### Forward Kinematics

For a serial chain with n joints, the complete transformation from base to end-effector is:

```
T_0^n = T_0^1 × T_1^2 × T_2^3 × ... × T_(n-1)^n
```

Where each T_(i-1)^i is computed using the DH parameters for joint i.

## DH Parameter Examples

### 2-DOF Planar Manipulator

Consider a simple 2-DOF planar manipulator:

| i | θ_i | d_i | a_i | α_i |
|---|-----|-----|-----|-----|
| 1 | θ₁  | 0   | a₁  | 0   |
| 2 | θ₂  | 0   | a₂  | 0   |

Where a₁ and a₂ are the link lengths.

The transformation matrices are:
```
T_0^1 = | cos(θ₁)  -sin(θ₁)  0  a₁cos(θ₁) |
        | sin(θ₁)   cos(θ₁)  0  a₁sin(θ₁) |
        | 0         0        1  0         |
        | 0         0        0  1         |

T_1^2 = | cos(θ₂)  -sin(θ₂)  0  a₂cos(θ₂) |
        | sin(θ₂)   cos(θ₂)  0  a₂sin(θ₂) |
        | 0         0        1  0         |
        | 0         0        0  1         |
```

### 3-DOF Spatial Manipulator

For a more complex example, consider a 3-DOF spatial manipulator:

| i | θ_i | d_i | a_i | α_i |
|---|-----|-----|-----|-----|
| 1 | θ₁  | d₁  | 0   | 90° |
| 2 | θ₂  | 0   | a₂  | 0   |
| 3 | θ₃  | 0   | a₃  | 0   |

## DH Parameter Derivation Process

### Step-by-Step Method

1. **Identify Joint Axes**: Locate and mark the joint axes (z₀, z₁, ..., zₙ)

2. **Assign z-axes**: Assign z_i along the axis of actuation for joint i+1

3. **Assign Origin**: Locate the origin o_i where the common normal to z_i and z_(i+1) intersects z_i

4. **Assign x-axis**: Define x_i along the common normal from z_i to z_(i+1)

5. **Assign y-axis**: Complete the right-handed system with y_i

6. **Define Parameters**: Measure the four DH parameters for each joint

### Common Challenges

- **Parallel Axes**: When z_i and z_(i+1) are parallel, the common normal is not unique
- **Intersecting Axes**: When z_i and z_(i+1) intersect, x_i can be chosen arbitrarily
- **Prismatic Joints**: For prismatic joints, θ_i is typically the variable instead of d_i

## Implementation in Robotics Software

### Python Implementation

```python
import numpy as np

class DHParameters:
    def __init__(self, dh_table):
        """
        Initialize with DH parameter table
        dh_table: list of [theta, d, a, alpha] for each joint
        """
        self.dh_table = dh_table

    def transformation_matrix(self, theta, d, a, alpha):
        """
        Calculate transformation matrix for given DH parameters
        """
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        T = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        return T

    def forward_kinematics(self, joint_angles):
        """
        Calculate forward kinematics using DH parameters
        """
        if len(joint_angles) != len(self.dh_table):
            raise ValueError("Number of joint angles must match DH table length")

        T_total = np.eye(4)  # Identity matrix

        for i, (theta, d, a, alpha) in enumerate(self.dh_table):
            # Update theta for variable joints (revolute joints)
            current_theta = theta + joint_angles[i] if isinstance(theta, (int, float)) else theta

            T_link = self.transformation_matrix(current_theta, d, a, alpha)
            T_total = T_total @ T_link

        return T_total

    def extract_position_orientation(self, transform_matrix):
        """
        Extract position and orientation from transformation matrix
        """
        position = transform_matrix[:3, 3]
        rotation_matrix = transform_matrix[:3, :3]

        # Convert rotation matrix to Euler angles (XYZ convention)
        sy = np.sqrt(rotation_matrix[0,0]**2 + rotation_matrix[1,0]**2)
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(rotation_matrix[2,1], rotation_matrix[2,2])
            y = np.arctan2(-rotation_matrix[2,0], sy)
            z = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0])
        else:
            x = np.arctan2(-rotation_matrix[1,2], rotation_matrix[1,1])
            y = np.arctan2(-rotation_matrix[2,0], sy)
            z = 0

        orientation = np.array([x, y, z])

        return position, orientation

# Example usage
if __name__ == "__main__":
    # DH parameters for a simple 3-DOF manipulator
    dh_table = [
        [0, 0.1, 0, np.pi/2],    # Joint 1
        [0, 0, 0.5, 0],          # Joint 2
        [0, 0, 0.3, 0]           # Joint 3
    ]

    robot = DHParameters(dh_table)
    joint_angles = [np.pi/4, np.pi/6, np.pi/3]  # 45°, 30°, 60°

    transform = robot.forward_kinematics(joint_angles)
    position, orientation = robot.extract_position_orientation(transform)

    print(f"End-effector position: {position}")
    print(f"End-effector orientation: {orientation}")
```

### C++ Implementation

```cpp
#include <vector>
#include <cmath>
#include <array>
#include <Eigen/Dense>

struct DHParameter {
    double theta;  // Joint angle (variable for revolute joints)
    double d;      // Link offset
    double a;      // Link length
    double alpha;  // Link twist

    DHParameter(double t, double offset, double length, double twist)
        : theta(t), d(offset), a(length), alpha(twist) {}
};

class DHRobotKinematics {
private:
    std::vector<DHParameter> dh_params_;

public:
    DHRobotKinematics(const std::vector<DHParameter>& dh_params)
        : dh_params_(dh_params) {}

    Eigen::Matrix4d getTransformationMatrix(double theta, double d, double a, double alpha) {
        double ct = cos(theta), st = sin(theta);
        double ca = cos(alpha), sa = sin(alpha);

        Eigen::Matrix4d T;
        T << ct, -st*ca, st*sa, a*ct,
             st, ct*ca, -ct*sa, a*st,
             0, sa, ca, d,
             0, 0, 0, 1;

        return T;
    }

    Eigen::Matrix4d forwardKinematics(const std::vector<double>& joint_angles) {
        if (joint_angles.size() != dh_params_.size()) {
            throw std::invalid_argument("Joint angles size must match DH parameters size");
        }

        Eigen::Matrix4d T_total = Eigen::Matrix4d::Identity();

        for (size_t i = 0; i < dh_params_.size(); ++i) {
            double current_theta = dh_params_[i].theta + joint_angles[i];
            Eigen::Matrix4d T_link = getTransformationMatrix(
                current_theta,
                dh_params_[i].d,
                dh_params_[i].a,
                dh_params_[i].alpha
            );
            T_total = T_total * T_link;
        }

        return T_total;
    }

    std::array<double, 3> getPosition(const Eigen::Matrix4d& transform) {
        return {transform(0, 3), transform(1, 3), transform(2, 3)};
    }

    void addDHParameter(double theta, double d, double a, double alpha) {
        dh_params_.emplace_back(theta, d, a, alpha);
    }
};
```

## Applications and Use Cases

### Robot Modeling

DH parameters are essential for:
- Creating accurate kinematic models
- Robot simulation and visualization
- Forward and inverse kinematics calculations
- Dynamics modeling

### Motion Planning

DH parameters enable:
- Trajectory generation in joint space
- Workspace analysis
- Singularity detection
- Path planning algorithms

### Control Systems

DH parameters support:
- Cartesian position control
- Jacobian-based control
- Visual servoing
- Force control

## Common DH Parameter Configurations

### Anthropomorphic Arm

A typical 6-DOF anthropomorphic arm might have DH parameters like:

| Joint | θ | d | a | α |
|-------|---|---|---|---|
| 1 | θ₁ | d₁ | 0 | π/2 |
| 2 | θ₂ | 0 | a₂ | 0 |
| 3 | θ₃ | 0 | a₃ | 0 |
| 4 | θ₄ | d₄ | 0 | π/2 |
| 5 | θ₅ | 0 | 0 | -π/2 |
| 6 | θ₆ | d₆ | 0 | 0 |

### SCARA Robot

A Selective Compliance Assembly Robot Arm (SCARA) configuration:

| Joint | θ | d | a | α |
|-------|---|---|---|---|
| 1 | θ₁ | d₁ | a₁ | 0 |
| 2 | θ₂ | 0 | a₂ | 0 |
| 3 | 0 | d₃ | 0 | 0 | (prismatic)
| 4 | θ₄ | 0 | 0 | 0 |

## Limitations and Alternatives

### Limitations of DH Parameters

- **Singular configurations**: DH parameters can become undefined for certain robot configurations
- **Complex joint types**: Not easily applicable to complex joint types beyond revolute and prismatic
- **Non-standard mechanisms**: Challenging to apply to parallel mechanisms or complex kinematic structures
- **Multiple solutions**: For some configurations, multiple DH parameter sets may exist

### Alternative Representations

- **Product of Exponentials (PoE)**: Uses screw theory for more general kinematic representation
- **Modified DH parameters**: Alternative parameterization that can handle certain singularities better
- **Natural representations**: Uses geometric properties of the mechanism directly

## Best Practices

### Parameter Selection

1. **Consistency**: Use the same DH convention throughout the robot model
2. **Documentation**: Clearly document the DH parameter table and frame assignments
3. **Validation**: Verify that the DH parameters produce expected kinematic behavior
4. **Testing**: Test with known configurations to validate the model

### Implementation Tips

- **Numerical stability**: Be careful with trigonometric calculations near singularities
- **Coordinate frame visualization**: Use visualization tools to verify frame assignments
- **Unit consistency**: Ensure consistent units throughout the parameter table
- **Joint limit integration**: Consider joint limits when implementing forward kinematics

## Troubleshooting Common Issues

### Frame Assignment Problems

- **Incorrect z-axis orientation**: Verify z-axes align with joint actuation axes
- **Wrong x-axis direction**: Ensure x-axes point along common normals
- **Inconsistent handedness**: Verify all frames follow the same handedness convention

### Mathematical Issues

- **Transformation matrix errors**: Double-check the mathematical formulation
- **Joint variable confusion**: Ensure the correct parameter is treated as variable
- **Sign convention errors**: Verify sign conventions match the intended model

## Integration with Robotics Frameworks

### ROS Integration

DH parameters can be integrated with ROS through URDF and kinematics plugins:

```xml
<!-- In URDF, DH parameters are implicitly defined through joint definitions -->
<joint name="joint1" type="revolute">
  <parent link="base_link"/>
  <child link="link1"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
</joint>
```

### MoveIt! Integration

MoveIt! can use DH parameters for kinematic calculations through custom kinematics plugins or by properly structuring the URDF to represent the DH structure.

## Advanced Topics

### Kinematic Calibration

DH parameters can be refined through kinematic calibration to account for manufacturing tolerances and assembly errors. This involves measuring actual robot positions and adjusting the DH parameters to minimize position errors.

### Dynamic Parameters

While DH parameters primarily address kinematics, they form the foundation for dynamic modeling using methods like the Newton-Euler or Lagrangian formulations.

### Redundant Manipulators

For robots with more than 6 DOF, DH parameters can still be used, but additional considerations for redundancy resolution become important.

## Conclusion

The Denavit-Hartenberg parameters provide a systematic and mathematically rigorous approach to modeling serial kinematic chains in robotics. Despite some limitations, the DH convention remains a fundamental tool in robotics education and practice due to its systematic approach and well-established methodology. Understanding DH parameters is essential for anyone working with robotic kinematics, as they form the basis for forward kinematics, trajectory planning, and robot control. Modern robotics frameworks often abstract away the DH parameters, but the underlying principles remain relevant for understanding robot kinematics and for developing custom kinematic solutions.
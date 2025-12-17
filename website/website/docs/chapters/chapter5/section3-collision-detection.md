---
sidebar_label: 'Collision Detection'
title: 'Collision Detection'
---

# Collision Detection

## Introduction

Collision detection is a fundamental computational problem in robotics, computer graphics, and physics simulation that determines when two or more objects intersect or come into contact. In robotics, collision detection is critical for safe navigation, manipulation, path planning, and human-robot interaction. The algorithms must efficiently identify potential collisions while maintaining real-time performance for dynamic robotic systems. Modern collision detection systems balance accuracy, performance, and robustness to ensure reliable robot operation in complex environments.

## Fundamentals of Collision Detection

### Problem Definition

Collision detection involves determining whether two geometric objects intersect:

**Static Collision Detection:**
- Determine if objects are currently intersecting
- Position-based query at a single time instant

**Dynamic Collision Detection:**
- Predict future collisions based on object motion
- Consider velocities and trajectories
- Essential for real-time safety systems

### Basic Concepts

**Collision Object Types:**
- **Convex Objects**: Any line segment between two points lies entirely within the object
- **Concave Objects**: Objects that are not convex, may have indentations
- **Compound Objects**: Combinations of simpler geometric primitives

**Contact Types:**
- **Intersection**: Objects penetrate each other
- **Contact**: Objects touch at a point or surface
- **Proximity**: Objects are close but not touching

## Collision Detection Pipeline

### Two-Phase Approach

Most collision detection systems use a two-phase approach:

**Broad Phase:**
- Quickly eliminate non-colliding pairs
- Spatial partitioning and bounding volumes
- Reduces number of detailed tests needed

**Narrow Phase:**
- Precise collision detection between potential pairs
- Geometric algorithms for exact intersection
- Contact point and normal calculation

### Hierarchical Collision Detection

```cpp
class CollisionDetectionSystem {
private:
    // Broad-phase collision detection
    std::unique_ptr<BroadPhase> broad_phase_;

    // Narrow-phase collision detection
    std::unique_ptr<NarrowPhase> narrow_phase_;

    // Collision objects and their bounding volumes
    std::vector<CollisionObject> collision_objects_;

public:
    std::vector<ContactManifold> detectCollisions() {
        // 1. Broad phase: identify potential collisions
        std::vector<std::pair<int, int>> potential_pairs =
            broad_phase_->computeOverlappingPairs(collision_objects_);

        // 2. Narrow phase: precise collision detection
        std::vector<ContactManifold> contacts;
        for (const auto& pair : potential_pairs) {
            ContactManifold manifold = narrow_phase_->computeContact(
                collision_objects_[pair.first],
                collision_objects_[pair.second]
            );

            if (!manifold.isEmpty()) {
                contacts.push_back(manifold);
            }
        }

        return contacts;
    }
};
```

## Broad-Phase Algorithms

### Spatial Partitioning

**Uniform Grid:**
- Divide space into uniform cubic cells
- Objects placed in relevant cells
- Efficient for uniformly distributed objects

```cpp
class UniformGridBroadPhase {
private:
    float cell_size_;
    std::unordered_map<int, std::vector<int>> grid_cells_; // cell_id -> object_ids

public:
    std::vector<std::pair<int, int>> computeOverlappingPairs(
        const std::vector<CollisionObject>& objects) {

        // Clear grid
        grid_cells_.clear();

        // Insert objects into grid cells
        for (size_t i = 0; i < objects.size(); ++i) {
            auto cell_ids = getGridCellsForObject(objects[i]);
            for (int cell_id : cell_ids) {
                grid_cells_[cell_id].push_back(i);
            }
        }

        // Find overlapping pairs
        std::vector<std::pair<int, int>> pairs;
        for (const auto& [cell_id, object_list] : grid_cells_) {
            for (size_t i = 0; i < object_list.size(); ++i) {
                for (size_t j = i + 1; j < object_list.size(); ++j) {
                    pairs.emplace_back(object_list[i], object_list[j]);
                }
            }
        }

        // Remove duplicates (objects in multiple cells)
        std::sort(pairs.begin(), pairs.end());
        pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());

        return pairs;
    }

private:
    std::vector<int> getGridCellsForObject(const CollisionObject& obj) {
        std::vector<int> cell_ids;
        // Calculate which grid cells the object's AABB overlaps
        // Implementation depends on object's bounding box
        return cell_ids;
    }
};
```

**Spatial Hashing:**
- Hash function maps 3D coordinates to hash table
- Memory efficient for sparse environments
- Good performance for dynamic scenes

### Hierarchical Data Structures

**Bounding Volume Hierarchies (BVH):**
- Organize objects in tree structure
- Each node contains bounding volume of children
- Prune branches that don't intersect

```cpp
struct BVHNode {
    BoundingBox bounding_box;
    std::unique_ptr<BVHNode> left_child;
    std::unique_ptr<BVHNode> right_child;
    int object_id;  // -1 if internal node

    bool isLeaf() const { return object_id != -1; }
};

class BoundingVolumeHierarchy {
private:
    std::unique_ptr<BVHNode> root_;

public:
    void build(const std::vector<CollisionObject>& objects) {
        // Build BVH from objects using top-down approach
        root_ = buildRecursive(objects, 0, objects.size());
    }

    std::vector<int> findCollisions(const CollisionObject& query_object) {
        std::vector<int> collisions;
        findCollisionsRecursive(root_.get(), query_object, collisions);
        return collisions;
    }

private:
    std::unique_ptr<BVHNode> buildRecursive(
        const std::vector<CollisionObject>& objects,
        size_t start, size_t end) {

        if (end - start == 1) {
            // Create leaf node
            auto node = std::make_unique<BVHNode>();
            node->bounding_box = objects[start].getBoundingBox();
            node->object_id = start;
            return node;
        }

        // Calculate bounding box for this node
        BoundingBox node_box;
        for (size_t i = start; i < end; ++i) {
            node_box.expand(objects[i].getBoundingBox());
        }

        // Partition objects (using longest axis heuristic)
        int split_axis = node_box.getLongestAxis();
        float split_pos = node_box.getCenter()[split_axis];

        auto partition_point = std::partition(
            objects.begin() + start,
            objects.begin() + end,
            [split_axis, split_pos](const CollisionObject& obj) {
                return obj.getCenter()[split_axis] < split_pos;
            }
        );

        size_t mid = partition_point - objects.begin();

        // Create internal node
        auto node = std::make_unique<BVHNode>();
        node->bounding_box = node_box;
        node->object_id = -1;
        node->left_child = buildRecursive(objects, start, mid);
        node->right_child = buildRecursive(objects, mid, end);

        return node;
    }

    void findCollisionsRecursive(
        BVHNode* node,
        const CollisionObject& query_object,
        std::vector<int>& collisions) {

        if (!node || !node->bounding_box.intersects(query_object.getBoundingBox())) {
            return;  // No intersection, prune branch
        }

        if (node->isLeaf()) {
            // Leaf node - actual object to test
            if (narrowPhaseTest(query_object, node->object_id)) {
                collisions.push_back(node->object_id);
            }
        } else {
            // Internal node - test children
            findCollisionsRecursive(node->left_child.get(), query_object, collisions);
            findCollisionsRecursive(node->right_child.get(), query_object, collisions);
        }
    }

    bool narrowPhaseTest(const CollisionObject& obj1, int obj2_id) {
        // Perform narrow-phase collision test
        return false; // Implementation depends on specific algorithm
    }
};
```

**Octrees:**
- Recursive subdivision of 3D space into 8 octants
- Good for spatially coherent objects
- Logarithmic query time complexity

## Narrow-Phase Algorithms

### Convex Object Detection

**Separating Axis Theorem (SAT):**
- Two convex objects are separated if there exists a plane that separates them
- Test axes perpendicular to face normals and edge cross products
- Provides collision normal and penetration depth

```cpp
class SATCollisionDetector {
public:
    struct CollisionResult {
        bool intersecting = false;
        float penetration_depth = 0.0f;
        Vector3 normal;
        std::vector<Vector3> contact_points;
    };

    CollisionResult detect(const ConvexHull& hull1, const ConvexHull& hull2) {
        CollisionResult result;

        // Test hull1 face normals
        for (const Vector3& normal : hull1.getFaceNormals()) {
            auto [min1, max1] = projectOntoAxis(hull1, normal);
            auto [min2, max2] = projectOntoAxis(hull2, normal);

            if (max1 < min2 || max2 < min1) {
                // No overlap on this axis - no collision
                return result;  // Already initialized with intersecting = false
            }

            // Calculate overlap
            float overlap = std::min(max1, max2) - std::max(min1, min2);
            if (overlap < result.penetration_depth || !result.intersecting) {
                result.penetration_depth = overlap;
                result.normal = normal;
                result.intersecting = true;
            }
        }

        // Test hull2 face normals
        for (const Vector3& normal : hull2.getFaceNormals()) {
            auto [min1, max1] = projectOntoAxis(hull1, normal);
            auto [min2, max2] = projectOntoAxis(hull2, normal);

            if (max1 < min2 || max2 < min1) {
                result.intersecting = false;
                return result;
            }

            float overlap = std::min(max1, max2) - std::max(min1, min2);
            if (overlap < result.penetration_depth) {
                result.penetration_depth = overlap;
                result.normal = normal;
            }
        }

        // Test edge cross products
        for (const Vector3& edge1 : hull1.getEdges()) {
            for (const Vector3& edge2 : hull2.getEdges()) {
                Vector3 cross_axis = edge1.cross(edge2).normalized();
                if (cross_axis.length() < 1e-6) continue; // Parallel edges

                auto [min1, max1] = projectOntoAxis(hull1, cross_axis);
                auto [min2, max2] = projectOntoAxis(hull2, cross_axis);

                if (max1 < min2 || max2 < min1) {
                    result.intersecting = false;
                    return result;
                }

                float overlap = std::min(max1, max2) - std::max(min1, min2);
                if (overlap < result.penetration_depth) {
                    result.penetration_depth = overlap;
                    result.normal = cross_axis;
                }
            }
        }

        // Calculate contact points using clipping
        result.contact_points = calculateContactPoints(hull1, hull2, result.normal);

        return result;
    }

private:
    std::pair<float, float> projectOntoAxis(const ConvexHull& hull, const Vector3& axis) {
        float min_proj = std::numeric_limits<float>::max();
        float max_proj = std::numeric_limits<float>::lowest();

        for (const Vector3& vertex : hull.getVertices()) {
            float proj = vertex.dot(axis);
            min_proj = std::min(min_proj, proj);
            max_proj = std::max(max_proj, proj);
        }

        return {min_proj, max_proj};
    }

    std::vector<Vector3> calculateContactPoints(
        const ConvexHull& hull1,
        const ConvexHull& hull2,
        const Vector3& normal) {
        // Simplified contact point calculation
        // In practice, this involves clipping faces against each other
        std::vector<Vector3> contacts;

        // Find support points in collision direction
        Vector3 support1 = hull1.getSupportPoint(normal);
        Vector3 support2 = hull2.getSupportPoint(-normal);

        // Calculate contact point as average
        Vector3 contact_point = (support1 + support2) * 0.5f;
        contacts.push_back(contact_point);

        return contacts;
    }
};
```

**Gilbert-Johnson-Keerthi (GJK) Algorithm:**
- Iterative algorithm for convex objects
- Uses Minkowski difference concept
- Robust and efficient for most convex shapes

```cpp
class GJKCollisionDetector {
public:
    struct GJKResult {
        bool intersecting = false;
        Vector3 closest_point_A;
        Vector3 closest_point_B;
        float distance = 0.0f;
    };

    GJKResult detect(const ConvexShape& shapeA, const ConvexShape& shapeB) {
        GJKResult result;

        // Start with a point in the Minkowski difference
        Vector3 simplex[4];  // Simplex can have max 4 points in 3D
        int simplex_size = 0;

        // Initial search direction
        Vector3 d = shapeA.getCenter() - shapeB.getCenter();
        if (d.length() < 1e-6) {
            d = Vector3(1, 0, 0);  // Arbitrary direction
        }

        for (int iteration = 0; iteration < 100; ++iteration) {
            // Get support point in direction d
            Vector3 pA = shapeA.getSupportPoint(d);
            Vector3 pB = shapeB.getSupportPoint(-d);
            Vector3 w = pA - pB;  // Point in Minkowski difference

            // If w points away from origin, no collision
            if (w.dot(d) < 0) {
                result.distance = std::sqrt(closestPtPointTriangle(Vector3(0,0,0), simplex, simplex_size));
                return result;
            }

            // Add w to simplex
            simplex[simplex_size++] = w;

            if (doSimplex(simplex, simplex_size, d)) {
                // Origin is inside simplex - intersection detected
                result.intersecting = true;

                // Calculate closest points (simplified)
                result.closest_point_A = pA;
                result.closest_point_B = pB;

                return result;
            }
        }

        return result;  // No intersection found within iteration limit
    }

private:
    bool doSimplex(Vector3* simplex, int& simplex_size, Vector3& d) {
        switch (simplex_size) {
            case 2:
                return lineCase(simplex, d);
            case 3:
                return triangleCase(simplex, d);
            case 4:
                return tetrahedronCase(simplex, d);
            default:
                return false;
        }
    }

    bool lineCase(Vector3* simplex, Vector3& d) {
        Vector3 A = simplex[1];  // Last added point
        Vector3 B = simplex[0];  // Previous point
        Vector3 AB = B - A;
        Vector3 AO = -A;

        if (AB.dot(AO) > 0) {
            d = tripleCrossProduct(AB, AO, AB);  // Perpendicular to AB towards origin
        } else {
            simplex[0] = A;  // Replace B with A
            d = AO;  // Search towards origin
        }

        return false;
    }

    bool triangleCase(Vector3* simplex, Vector3& d) {
        Vector3 A = simplex[2];  // Last added point
        Vector3 B = simplex[1];
        Vector3 C = simplex[0];

        Vector3 AB = B - A;
        Vector3 AC = C - A;
        Vector3 AO = -A;

        Vector3 ABC = AB.cross(AC);

        // Check if origin is above or below triangle
        if ((ABC.cross(AC)).dot(AO) > 0) {
            if (AC.dot(AO) > 0) {
                // Origin is in region of AC
                simplex[1] = A;  // Remove B
                simplex[0] = C;
                simplex[2] = A;  // New A at end
                d = tripleCrossProduct(AC, AO, AC);
                return false;
            } else {
                if (AB.dot(AO) > 0) {
                    // Origin is in region of AB
                    simplex[0] = B;  // Remove C
                    simplex[1] = A;  // New A at end
                    d = tripleCrossProduct(AB, AO, AB);
                    return false;
                } else {
                    simplex[0] = A;  // Remove B and C
                    d = AO;
                    return false;
                }
            }
        } else {
            if ((AB.cross(ABC)).dot(AO) > 0) {
                if (AB.dot(AO) > 0) {
                    simplex[0] = B;  // Remove C
                    simplex[1] = A;  // New A at end
                    d = tripleCrossProduct(AB, AO, AB);
                    return false;
                } else {
                    simplex[0] = A;  // Remove B and C
                    d = AO;
                    return false;
                }
            } else {
                if (AC.dot(AO) > 0) {
                    simplex[1] = A;  // Remove B
                    simplex[0] = C;
                    simplex[2] = A;  // New A at end
                    d = tripleCrossProduct(AC, AO, AC);
                    return false;
                } else {
                    simplex[0] = A;  // Remove B and C
                    d = AO;
                    return false;
                }
            }
        }
    }

    bool tetrahedronCase(Vector3* simplex, Vector3& d) {
        Vector3 A = simplex[3];  // Last added point
        Vector3 B = simplex[2];
        Vector3 C = simplex[1];
        Vector3 D = simplex[0];

        Vector3 AO = -A;
        Vector3 AB = B - A;
        Vector3 AC = C - A;
        Vector3 AD = D - A;

        Vector3 ABC = AB.cross(AC);
        Vector3 ACD = AC.cross(AD);
        Vector3 ADB = AD.cross(AB);

        // Check which side of each face the origin is on
        if (ABC.dot(AO) > 0) {
            // Origin is in direction of ABC normal
            simplex[0] = A;  // Remove D
            simplex[1] = C;
            simplex[2] = B;
            simplex[3] = A;  // New A at end
            return false;
        }

        if (ACD.dot(AO) > 0) {
            // Origin is in direction of ACD normal
            simplex[0] = A;  // Remove B
            simplex[1] = D;
            simplex[2] = C;
            simplex[3] = A;  // New A at end
            return false;
        }

        if (ADB.dot(AO) > 0) {
            // Origin is in direction of ADB normal
            simplex[0] = A;  // Remove C
            simplex[1] = B;
            simplex[2] = D;
            simplex[3] = A;  // New A at end
            return false;
        }

        // Origin is inside tetrahedron - intersection found!
        return true;
    }

    Vector3 tripleCrossProduct(const Vector3& a, const Vector3& b, const Vector3& c) {
        return b * a.dot(c) - c * a.dot(b);
    }

    float closestPtPointTriangle(const Vector3& p, Vector3* simplex, int size) {
        // Simplified implementation - in practice, this calculates
        // the closest point on a triangle to a point
        return 0.0f;
    }
};
```

### Primitive Collision Tests

**Sphere-Sphere:**
```cpp
bool testSphereSphere(const Sphere& s1, const Sphere& s2) {
    Vector3 center_diff = s1.center - s2.center;
    float distance_sq = center_diff.lengthSquared();
    float radius_sum = s1.radius + s2.radius;
    return distance_sq <= (radius_sum * radius_sum);
}
```

**Sphere-Box:**
```cpp
bool testSphereBox(const Sphere& sphere, const Box& box) {
    // Find closest point on box to sphere center
    Vector3 closest_point;
    closest_point.x = std::clamp(sphere.center.x, box.min.x, box.max.x);
    closest_point.y = std::clamp(sphere.center.y, box.min.y, box.max.y);
    closest_point.z = std::clamp(sphere.center.z, box.min.z, box.max.z);

    Vector3 diff = sphere.center - closest_point;
    return diff.lengthSquared() <= (sphere.radius * sphere.radius);
}
```

## Continuous Collision Detection

### Problem with Discrete Detection

Discrete collision detection can miss collisions when objects move quickly between frames (tunneling effect).

### Conservative Advancement

```cpp
class ContinuousCollisionDetector {
public:
    struct CCDResult {
        bool collision = false;
        float time_of_impact = 1.0f;  // 0.0 = now, 1.0 = end of step
        Vector3 collision_point;
        Vector3 collision_normal;
    };

    CCDResult detectContinuous(
        const CollisionObject& obj1,
        const Vector3& vel1,
        const CollisionObject& obj2,
        const Vector3& vel2,
        float dt) {

        CCDResult result;

        // Relative velocity
        Vector3 rel_vel = vel1 - vel2;
        float rel_speed = rel_vel.length();

        if (rel_speed < 1e-6) {
            // No relative motion, use discrete detection
            if (testDiscreteCollision(obj1, obj2)) {
                result.collision = true;
                result.time_of_impact = 0.0f;
            }
            return result;
        }

        Vector3 dir = rel_vel.normalized();

        // Iteratively advance the collision
        float t = 0.0f;
        float t_remaining = dt;

        while (t < dt) {
            // Advance objects by small step
            float step_size = calculateConservativeStep(obj1, obj2, dir);

            if (step_size < 1e-6) {
                // Very close - likely colliding
                result.collision = true;
                result.time_of_impact = t / dt;
                result.collision_point = (obj1.getCenter() + obj2.getCenter()) * 0.5f;
                result.collision_normal = dir;
                return result;
            }

            // Check if step would exceed remaining time
            step_size = std::min(step_size, t_remaining);

            // Move objects
            Vector3 disp = rel_vel * (step_size / rel_speed);
            obj1.translate(disp);
            obj2.translate(-disp);

            t += step_size;
            t_remaining -= step_size;

            // Check for collision
            if (testDiscreteCollision(obj1, obj2)) {
                result.collision = true;
                result.time_of_impact = t / dt;
                result.collision_point = (obj1.getCenter() + obj2.getCenter()) * 0.5f;
                result.collision_normal = dir;
                return result;
            }
        }

        return result;
    }

private:
    float calculateConservativeStep(
        const CollisionObject& obj1,
        const CollisionObject& obj2,
        const Vector3& dir) {

        // Calculate conservative advancement step
        // This is a simplified version - real implementation would use
        // closest points and relative motion
        float dist = (obj1.getCenter() - obj2.getCenter()).length();
        float min_size = std::min(obj1.getMinimumSize(), obj2.getMinimumSize());

        return dist - (obj1.getRadius() + obj2.getRadius());
    }

    bool testDiscreteCollision(const CollisionObject& obj1, const CollisionObject& obj2) {
        // Implementation depends on object types
        return false;
    }
};
```

## Collision Response

### Impulse-Based Response

```cpp
class CollisionResponse {
public:
    struct ContactData {
        Vector3 normal;
        Vector3 point;
        float penetration_depth;
        float restitution;
        float friction;
    };

    void resolveCollision(
        RigidBody& body1,
        RigidBody& body2,
        const ContactData& contact) {

        // Relative velocity at contact point
        Vector3 r1 = contact.point - body1.position;
        Vector3 r2 = contact.point - body2.position;

        Vector3 vel1 = body1.velocity + body1.angular_velocity.cross(r1);
        Vector3 vel2 = body2.velocity + body2.angular_velocity.cross(r2);
        Vector3 rel_vel = vel1 - vel2;

        // Normal impulse
        float vel_along_normal = rel_vel.dot(contact.normal);

        if (vel_along_normal > 0) {
            // Objects separating, no collision response needed
            return;
        }

        // Calculate impulse magnitude
        float e = std::min(contact.restitution, body1.restitution * body2.restitution);
        float j = -(1 + e) * vel_along_normal;

        float mass_sum = body1.inv_mass + body2.inv_mass;
        Vector3 r1_cross_n = r1.cross(contact.normal);
        Vector3 r2_cross_n = r2.cross(contact.normal);

        float rn1 = r1_cross_n.dot(r1_cross_n) * body1.inv_inertia_tensor;
        float rn2 = r2_cross_n.dot(r2_cross_n) * body2.inv_inertia_tensor;

        j /= mass_sum + rn1 + rn2;

        Vector3 impulse = contact.normal * j;

        // Apply impulse
        body1.velocity += impulse * body1.inv_mass;
        body1.angular_velocity += r1.cross(impulse) * body1.inv_inertia_tensor;

        body2.velocity -= impulse * body2.inv_mass;
        body2.angular_velocity -= r2.cross(impulse) * body2.inv_inertia_tensor;

        // Apply friction
        resolveFriction(body1, body2, contact, impulse);

        // Correct position penetration
        correctPosition(body1, body2, contact);
    }

private:
    void resolveFriction(
        RigidBody& body1,
        RigidBody& body2,
        const ContactData& contact,
        const Vector3& normal_impulse) {

        // Tangential velocity
        Vector3 tangential_vel = rel_vel - contact.normal * vel_along_normal;
        float tangential_speed = tangential_vel.length();

        if (tangential_speed < 1e-6) return;

        Vector3 tangent = tangential_vel.normalized();
        float friction_coeff = std::sqrt(body1.friction * body2.friction);

        // Calculate friction impulse
        float jt = -(1 + e) * tangential_vel.dot(tangent);
        jt /= mass_sum + rt1 + rt2;  // rt1, rt2 similar to rn1, rn2 but for tangent

        // Clamp friction impulse
        float max_friction = friction_coeff * normal_impulse.length();
        jt = std::clamp(jt, -max_friction, max_friction);

        Vector3 friction_impulse = tangent * jt;

        // Apply friction impulse
        body1.velocity += friction_impulse * body1.inv_mass;
        body1.angular_velocity += r1.cross(friction_impulse) * body1.inv_inertia_tensor;

        body2.velocity -= friction_impulse * body2.inv_mass;
        body2.angular_velocity -= r2.cross(friction_impulse) * body2.inv_inertia_tensor;
    }

    void correctPosition(
        RigidBody& body1,
        RigidBody& body2,
        const ContactData& contact) {

        float percent = 0.8f;  // Percentage to correct
        float slop = 0.01f;    // Penetration tolerance

        float correction = std::max(contact.penetration_depth - slop, 0.0f) /
                          (body1.inv_mass + body2.inv_mass);

        Vector3 correction_vector = contact.normal * correction * percent;

        body1.position += correction_vector * body1.inv_mass;
        body2.position -= correction_vector * body2.inv_mass;
    }
};
```

## Implementation in Robotics Frameworks

### Integration with ROS/Gazebo

```xml
<!-- Example URDF with collision properties -->
<robot name="collision_robot">
  <link name="base_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="base_visual.dae"/>
      </geometry>
    </visual>
  </link>

  <link name="arm_link">
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

### Bullet Physics Integration

```cpp
#include "btBulletDynamicsCommon.h"

class RobotCollisionManager {
private:
    btDiscreteDynamicsWorld* dynamics_world_;
    std::vector<btRigidBody*> robot_links_;

public:
    void setupRobotCollision(const std::vector<LinkGeometry>& link_geometries) {
        for (const auto& geom : link_geometries) {
            btCollisionShape* shape = nullptr;

            switch (geom.type) {
                case GeometryType::BOX:
                    shape = new btBoxShape(btVector3(geom.size.x/2, geom.size.y/2, geom.size.z/2));
                    break;
                case GeometryType::SPHERE:
                    shape = new btSphereShape(geom.radius);
                    break;
                case GeometryType::CYLINDER:
                    shape = new btCylinderShape(btVector3(geom.radius, geom.length/2, geom.radius));
                    break;
                case GeometryType::MESH:
                    // Create from triangle mesh
                    shape = createMeshShape(geom.mesh_data);
                    break;
            }

            btTransform transform;
            transform.setIdentity();
            transform.setOrigin(btVector3(geom.position.x, geom.position.y, geom.position.z));

            btScalar mass = geom.mass;
            btVector3 local_inertia(0, 0, 0);
            if (mass != 0.0f) {
                shape->calculateLocalInertia(mass, local_inertia);
            }

            btDefaultMotionState* motion_state = new btDefaultMotionState(transform);
            btRigidBody::btRigidBodyConstructionInfo rb_info(mass, motion_state, shape, local_inertia);

            btRigidBody* body = new btRigidBody(rb_info);
            dynamics_world_->addRigidBody(body);

            robot_links_.push_back(body);
        }
    }

    bool checkSelfCollision() {
        // Get overlapping pairs from broadphase
        btDispatcher* dispatcher = dynamics_world_->getDispatcher();
        int num_manifolds = dispatcher->getNumManifolds();

        for (int i = 0; i < num_manifolds; ++i) {
            btPersistentManifold* contact_manifold = dispatcher->getManifoldByIndexInternal(i);

            const btCollisionObject* obj0 = contact_manifold->getBody0();
            const btCollisionObject* obj1 = contact_manifold->getBody1();

            // Check if both objects belong to the same robot
            if (isRobotLink(obj0) && isRobotLink(obj1)) {
                int num_contacts = contact_manifold->getNumContacts();
                for (int j = 0; j < num_contacts; ++j) {
                    btManifoldPoint& pt = contact_manifold->getContactPoint(j);
                    if (pt.getDistance() < 0) {  // Negative distance = penetration
                        return true;  // Self-collision detected
                    }
                }
            }
        }

        return false;
    }

private:
    bool isRobotLink(const btCollisionObject* obj) {
        auto it = std::find(robot_links_.begin(), robot_links_.end(), obj);
        return it != robot_links_.end();
    }

    btCollisionShape* createMeshShape(const MeshData& mesh_data) {
        btTriangleMesh* triangle_mesh = new btTriangleMesh();

        for (size_t i = 0; i < mesh_data.indices.size(); i += 3) {
            btVector3 v0(
                mesh_data.vertices[mesh_data.indices[i]].x,
                mesh_data.vertices[mesh_data.indices[i]].y,
                mesh_data.vertices[mesh_data.indices[i]].z
            );
            btVector3 v1(
                mesh_data.vertices[mesh_data.indices[i+1]].x,
                mesh_data.vertices[mesh_data.indices[i+1]].y,
                mesh_data.vertices[mesh_data.indices[i+1]].z
            );
            btVector3 v2(
                mesh_data.vertices[mesh_data.indices[i+2]].x,
                mesh_data.vertices[mesh_data.indices[i+2]].y,
                mesh_data.vertices[mesh_data.indices[i+2]].z
            );

            triangle_mesh->addTriangle(v0, v1, v2);
        }

        btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(triangle_mesh, true);
        return shape;
    }
};
```

## Performance Optimization

### Spatial Coherence

```cpp
class CoherentCollisionSystem {
private:
    std::vector<int> previous_collisions_;
    std::vector<int> new_collisions_;

public:
    std::vector<ContactManifold> detectCollisionsWithCoherence() {
        // First, test previously colliding pairs (likely still colliding)
        std::vector<ContactManifold> contacts;

        for (int pair_id : previous_collisions_) {
            auto [obj1_id, obj2_id] = decodePairID(pair_id);
            if (quickRejectionTest(obj1_id, obj2_id)) {
                // Still potentially colliding, do full test
                ContactManifold manifold = narrowPhaseTest(obj1_id, obj2_id);
                if (!manifold.isEmpty()) {
                    contacts.push_back(manifold);
                    new_collisions_.push_back(pair_id);
                }
            }
            // If quick rejection fails, pair is no longer colliding
        }

        // Then, do broad phase for new potential collisions
        auto new_pairs = broadPhaseDetection();
        for (auto pair : new_pairs) {
            if (std::find(previous_collisions_.begin(), previous_collisions_.end(),
                         encodePairID(pair)) == previous_collisions_.end()) {
                // This is a new potential collision
                ContactManifold manifold = narrowPhaseTest(pair.first, pair.second);
                if (!manifold.isEmpty()) {
                    contacts.push_back(manifold);
                    new_collisions_.push_back(encodePairID(pair));
                }
            }
        }

        previous_collisions_ = new_collisions_;
        new_collisions_.clear();

        return contacts;
    }

private:
    bool quickRejectionTest(int obj1_id, int obj2_id) {
        // Fast bounding volume test
        return getBoundingVolume(obj1_id).intersects(getBoundingVolume(obj2_id));
    }

    int encodePairID(const std::pair<int, int>& pair) {
        // Encode pair as single integer for fast lookup
        return (std::min(pair.first, pair.second) << 16) | std::max(pair.first, pair.second);
    }

    std::pair<int, int> decodePairID(int encoded) {
        int obj1 = encoded >> 16;
        int obj2 = encoded & 0xFFFF;
        return {obj1, obj2};
    }
};
```

### Parallel Processing

```cpp
#include <thread>
#include <future>

class ParallelCollisionDetector {
private:
    std::vector<std::thread> worker_threads_;
    int num_threads_;

public:
    std::vector<ContactManifold> detectCollisionsParallel(
        const std::vector<CollisionObject>& objects,
        const std::vector<std::pair<int, int>>& candidate_pairs) {

        int pairs_per_thread = candidate_pairs.size() / num_threads_;
        std::vector<std::future<std::vector<ContactManifold>>> futures;

        for (int i = 0; i < num_threads_; ++i) {
            int start = i * pairs_per_thread;
            int end = (i == num_threads_ - 1) ? candidate_pairs.size() : start + pairs_per_thread;

            auto future = std::async(std::launch::async, [this, &objects, &candidate_pairs, start, end]() {
                std::vector<ContactManifold> thread_contacts;

                for (int j = start; j < end; ++j) {
                    ContactManifold manifold = narrowPhaseTest(
                        objects[candidate_pairs[j].first],
                        objects[candidate_pairs[j].second]
                    );

                    if (!manifold.isEmpty()) {
                        thread_contacts.push_back(manifold);
                    }
                }

                return thread_contacts;
            });

            futures.push_back(std::move(future));
        }

        // Collect results
        std::vector<ContactManifold> all_contacts;
        for (auto& future : futures) {
            auto thread_contacts = future.get();
            all_contacts.insert(all_contacts.end(),
                              thread_contacts.begin(),
                              thread_contacts.end());
        }

        return all_contacts;
    }
};
```

## Applications in Robotics

### Motion Planning

Collision detection is essential for motion planning algorithms:

**RRT (Rapidly-exploring Random Trees):**
```cpp
class RRTPlanner {
private:
    CollisionDetector* collision_detector_;

public:
    bool isValidConfiguration(const RobotState& state) {
        // Check if robot configuration is collision-free
        auto robot_links = computeRobotPoses(state);
        for (const auto& link : robot_links) {
            if (collision_detector_->checkCollision(link, environment_)) {
                return false;  // Invalid configuration
            }
        }
        return true;
    }

    bool isValidPathSegment(const RobotState& start, const RobotState& end) {
        // Check path for collisions using interpolation
        int num_samples = 10;  // Adjust based on path length
        for (int i = 1; i < num_samples; ++i) {
            float t = static_cast<float>(i) / num_samples;
            RobotState intermediate = interpolate(start, end, t);
            if (!isValidConfiguration(intermediate)) {
                return false;  // Path has collision
            }
        }
        return true;
    }
};
```

### Navigation

For mobile robots:
- Static obstacle collision detection
- Dynamic obstacle avoidance
- Safe path following

### Manipulation

For robotic arms:
- Self-collision avoidance
- Environment collision detection
- Grasp planning and execution

## Challenges and Limitations

### Computational Complexity

**Time Complexity:**
- Broad phase: O(n) to O(n log n)
- Narrow phase: O(1) to O(n) depending on algorithm
- Overall: O(n²) worst case without spatial partitioning

### Accuracy vs. Performance Trade-offs

**Simplification Strategies:**
- Bounding volume hierarchies
- Level of detail (LOD) for collision geometry
- Approximate algorithms for real-time applications

### Numerical Robustness

**Floating-Point Issues:**
- Precision errors in geometric calculations
- Robust geometric predicates
- Tolerance-based comparisons

## Best Practices

### Geometry Optimization

**Collision Geometry Selection:**
- Use simple primitives when possible
- Balance accuracy with performance
- Consider both visual and collision geometry

**Hierarchical Approaches:**
- Quick rejection with bounding volumes
- Progressive refinement
- Cache previous results

### Algorithm Selection

**Choose Appropriate Algorithms:**
- GJK for convex objects
- SAT for polyhedral objects
- Specialized tests for primitives

### Validation and Testing

**Comprehensive Testing:**
- Unit tests for individual components
- Integration tests with complex scenarios
- Performance benchmarking

## Future Trends

### Machine Learning Integration

**Learning-Based Collision Detection:**
- Neural networks for collision prediction
- Learned collision features
- Adaptive algorithm selection

### GPU Acceleration

**Parallel Collision Detection:**
- CUDA/OpenCL implementations
- Massive parallel processing
- Real-time performance for large scenes

### Advanced Techniques

**Deformable Object Detection:**
- Soft body collision handling
- FEM-based deformation
- Real-time performance

## Conclusion

Collision detection is a fundamental component of robotic systems, enabling safe navigation, manipulation, and interaction. Modern collision detection systems employ sophisticated algorithms and data structures to balance accuracy, performance, and robustness. The two-phase approach with broad and narrow phases remains the standard, with various algorithms optimized for different types of objects and applications. As robotics applications become more complex and demanding, collision detection continues to evolve with parallel processing, GPU acceleration, and machine learning techniques to meet the growing requirements of real-time robotic systems. The choice of collision detection approach should consider the specific requirements of the application, including accuracy needs, performance constraints, and the types of objects involved.
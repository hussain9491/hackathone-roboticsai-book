---
sidebar_label: 'Sensor Simulation'
title: 'Sensor Simulation'
---

# Sensor Simulation

## Introduction

Sensor simulation is a critical component of robotics development that enables the testing and validation of perception algorithms, navigation systems, and control strategies in virtual environments before deployment on physical robots. By accurately simulating the behavior of real-world sensors in a controlled virtual environment, developers can evaluate robot performance, test edge cases, and validate algorithms without the risks and costs associated with physical testing. Modern sensor simulation systems incorporate sophisticated physical models to reproduce the characteristics, limitations, and noise patterns of real sensors, bridging the reality gap between simulation and real-world performance.

## Fundamentals of Sensor Simulation

### Core Principles

Sensor simulation operates on the principle of reproducing the physical processes that occur when real sensors interact with their environment:

**Ray Tracing Model:**
- Simulates the path of electromagnetic radiation (light, radio waves, etc.)
- Calculates interactions with surfaces and materials
- Reproduces realistic reflections, refractions, and absorptions

**Physical Property Mapping:**
- Maps virtual environment properties to sensor responses
- Accounts for material properties, lighting conditions, and environmental factors
- Incorporates sensor-specific characteristics and limitations

**Noise and Distortion Modeling:**
- Simulates sensor-specific noise patterns
- Reproduces systematic distortions and imperfections
- Models environmental interference and artifacts

### Simulation Pipeline

The sensor simulation pipeline typically follows these stages:

1. **Scene Rendering:** Render the virtual environment from the sensor's perspective
2. **Physical Simulation:** Apply physical models for sensor-environment interaction
3. **Noise Injection:** Add realistic sensor noise and artifacts
4. **Post-Processing:** Apply sensor-specific processing and calibration
5. **Output Generation:** Generate sensor data in the appropriate format

## Camera Simulation

### Pinhole Camera Model

The pinhole camera model forms the foundation for camera simulation:

```cpp
class CameraSimulator {
private:
    float fx_, fy_;  // Focal lengths in pixels
    float cx_, cy_;  // Principal point coordinates
    float k1_, k2_, p1_, p2_, k3_;  // Distortion coefficients
    int width_, height_;  // Image dimensions

public:
    cv::Mat simulateImage(const Scene& scene, const Transform& camera_pose) {
        cv::Mat image(height_, width_, CV_8UC3);

        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                // Convert pixel coordinates to normalized coordinates
                float x_norm = (x - cx_) / fx_;
                float y_norm = (y - cy_) / fy_;

                // Apply distortion
                std::tie(x_norm, y_norm) = applyDistortion(x_norm, y_norm);

                // Calculate ray direction in camera frame
                Eigen::Vector3f ray_dir(x_norm, y_norm, 1.0f);

                // Transform ray to world frame
                Eigen::Vector3f world_ray = camera_pose.rotation * ray_dir;
                Eigen::Vector3f world_origin = camera_pose.translation;

                // Ray-scene intersection
                IntersectionResult hit = scene.rayIntersect(world_origin, world_ray);

                if (hit.valid) {
                    // Calculate pixel color based on material properties and lighting
                    cv::Vec3b pixel_color = calculatePixelColor(hit, scene);
                    image.at<cv::Vec3b>(y, x) = pixel_color;
                } else {
                    // Background color (sky, etc.)
                    image.at<cv::Vec3b>(y, x) = cv::Vec3b(135, 206, 235); // Sky blue
                }
            }
        }

        // Add sensor noise
        addNoise(image);

        return image;
    }

private:
    std::pair<float, float> applyDistortion(float x, float y) {
        float r2 = x*x + y*y;
        float r4 = r2 * r2;
        float r6 = r4 * r2;

        // Radial distortion
        float radial_factor = 1.0f + k1_*r2 + k2_*r4 + k3_*r6;

        // Tangential distortion
        float dx = 2*p1_*x*y + p2_*(r2 + 2*x*x);
        float dy = p1_*(r2 + 2*y*y) + 2*p2_*x*y;

        float x_distorted = x * radial_factor + dx;
        float y_distorted = y * radial_factor + dy;

        return {x_distorted, y_distorted};
    }

    cv::Vec3b calculatePixelColor(const IntersectionResult& hit, const Scene& scene) {
        // Calculate color based on:
        // - Material properties at intersection point
        // - Lighting conditions
        // - Surface normal and viewing angle
        // - Shading model (Phong, Blinn-Phong, etc.)

        // Simplified calculation
        cv::Vec3b base_color = hit.material.color;

        // Apply lighting based on surface normal and light sources
        float intensity = calculateLightingIntensity(hit.point, hit.normal, scene);

        return base_color * intensity;
    }

    float calculateLightingIntensity(const Eigen::Vector3f& point,
                                   const Eigen::Vector3f& normal,
                                   const Scene& scene) {
        float intensity = scene.ambient_light;

        for (const auto& light : scene.lights) {
            Eigen::Vector3f light_dir = (light.position - point).normalized();

            // Diffuse lighting
            float diffuse = std::max(0.0f, normal.dot(light_dir));

            // Specular lighting (simplified)
            Eigen::Vector3f view_dir = (scene.camera_position - point).normalized();
            Eigen::Vector3f reflect_dir = light_dir - 2.0f * normal.dot(light_dir) * normal;
            float specular = pow(std::max(0.0f, view_dir.dot(reflect_dir)), 32.0f);

            intensity += light.intensity * (diffuse + 0.1f * specular);
        }

        return std::min(1.0f, intensity);
    }

    void addNoise(cv::Mat& image) {
        cv::Mat noise = cv::Mat::zeros(image.size(), image.type());
        cv::randn(noise, cv::Scalar::all(0), cv::Scalar::all(10));  // Gaussian noise
        cv::add(image, noise, image);
    }
};
```

### Stereo Camera Simulation

Stereo vision simulation requires synchronized simulation of two cameras:

```cpp
class StereoCameraSimulator {
private:
    CameraSimulator left_camera_;
    CameraSimulator right_camera_;
    Eigen::Vector3f baseline_;  // Translation between cameras

public:
    struct StereoOutput {
        cv::Mat left_image;
        cv::Mat right_image;
        cv::Mat disparity_map;
        cv::Mat depth_map;
    };

    StereoOutput simulateStereo(const Scene& scene,
                               const Transform& left_pose) {
        Transform right_pose = left_pose;
        right_pose.translation += baseline_;

        StereoOutput output;
        output.left_image = left_camera_.simulateImage(scene, left_pose);
        output.right_image = right_camera_.simulateImage(scene, right_pose);

        // Generate disparity map by comparing corresponding pixels
        output.disparity_map = computeDisparity(output.left_image, output.right_image);

        // Convert disparity to depth
        output.depth_map = disparityToDepth(output.disparity_map);

        return output;
    }

private:
    cv::Mat computeDisparity(const cv::Mat& left, const cv::Mat& right) {
        // Simplified disparity computation
        // In practice, this would use stereo matching algorithms

        cv::Mat disparity(left.rows, left.cols, CV_32F);

        for (int y = 0; y < left.rows; ++y) {
            for (int x = 0; x < left.cols; ++x) {
                // Search along scan line for matching features
                float best_match = 0;
                int best_disparity = 0;

                for (int d = 0; d < 64 && x - d >= 0; ++d) {  // Search range
                    // Compare windows around pixels
                    float similarity = compareWindows(left, right, x, y, x - d, y);

                    if (similarity > best_match) {
                        best_match = similarity;
                        best_disparity = d;
                    }
                }

                disparity.at<float>(y, x) = best_disparity;
            }
        }

        return disparity;
    }

    cv::Mat disparityToDepth(const cv::Mat& disparity) {
        cv::Mat depth(disparity.size(), CV_32F);

        for (int y = 0; y < disparity.rows; ++y) {
            for (int x = 0; x < disparity.cols; ++x) {
                float disp = disparity.at<float>(y, x);
                if (disp > 0) {
                    depth.at<float>(y, x) = (fx_ * baseline_.x()) / disp;  // Depth = (f * B) / disparity
                } else {
                    depth.at<float>(y, x) = 0;  // Invalid depth
                }
            }
        }

        return depth;
    }

    float compareWindows(const cv::Mat& img1, const cv::Mat& img2,
                        int x1, int y1, int x2, int y2, int window_size = 5) {
        // Calculate similarity between windows using SSD or NCC
        float sum_diff = 0;
        int count = 0;

        for (int dy = -window_size/2; dy <= window_size/2; ++dy) {
            for (int dx = -window_size/2; dx <= window_size/2; ++dx) {
                if (x1+dx >= 0 && x1+dx < img1.cols && y1+dy >= 0 && y1+dy < img1.rows &&
                    x2+dx >= 0 && x2+dx < img2.cols && y2+dy >= 0 && y2+dy < img2.rows) {

                    cv::Vec3b p1 = img1.at<cv::Vec3b>(y1+dy, x1+dx);
                    cv::Vec3b p2 = img2.at<cv::Vec3b>(y2+dy, x2+dx);

                    float diff = cv::norm(p1, p2);
                    sum_diff += diff;
                    count++;
                }
            }
        }

        return count > 0 ? 1.0f / (1.0f + sum_diff / count) : 0;  // Higher similarity = higher value
    }
};
```

## LiDAR Simulation

### Ray-Casting LiDAR Model

LiDAR simulation models the time-of-flight measurement process:

```cpp
class LiDARSimulator {
private:
    float fov_horizontal_;    // Horizontal field of view (degrees)
    float fov_vertical_;      // Vertical field of view (degrees)
    int num_rays_horizontal_; // Number of horizontal rays
    int num_rays_vertical_;   // Number of vertical rays
    float max_range_;         // Maximum detection range
    float min_range_;         // Minimum detection range
    float noise_std_dev_;     // Noise standard deviation

public:
    struct LiDARData {
        std::vector<float> ranges;
        std::vector<Eigen::Vector3f> points;
        std::vector<float> intensities;
        double timestamp;
    };

    LiDARData simulateScan(const Scene& scene, const Transform& lidar_pose) {
        LiDARData data;
        data.timestamp = getCurrentTime();

        float h_angle_step = fov_horizontal_ / (num_rays_horizontal_ - 1);
        float v_angle_step = fov_vertical_ / (num_rays_vertical_ - 1);

        float h_start = -fov_horizontal_/2.0f;
        float v_start = -fov_vertical_/2.0f;

        for (int v_idx = 0; v_idx < num_rays_vertical_; ++v_idx) {
            float v_angle = v_start + v_idx * v_angle_step;
            float v_cos = cos(deg2rad(v_angle));
            float v_sin = sin(deg2rad(v_angle));

            for (int h_idx = 0; h_idx < num_rays_horizontal_; ++h_idx) {
                float h_angle = h_start + h_idx * h_angle_step;
                float h_cos = cos(deg2rad(h_angle));
                float h_sin = sin(deg2rad(h_angle));

                // Calculate ray direction in LiDAR frame
                Eigen::Vector3f ray_dir_local(
                    v_cos * h_cos,  // x
                    v_cos * h_sin,  // y
                    v_sin           // z
                );

                // Transform to world frame
                Eigen::Vector3f ray_dir_world = lidar_pose.rotation * ray_dir_local;
                Eigen::Vector3f ray_origin = lidar_pose.translation;

                // Ray intersection test
                IntersectionResult hit = scene.rayIntersect(ray_origin, ray_dir_world);

                float range = max_range_;  // Default to max range (no detection)
                Eigen::Vector3f point(0, 0, 0);
                float intensity = 0.0f;

                if (hit.valid && hit.distance <= max_range_) {
                    range = hit.distance;
                    point = hit.point;
                    intensity = calculateIntensity(hit);
                }

                // Add noise to range measurement
                range += generateNoise();

                // Store data
                data.ranges.push_back(range);
                data.points.push_back(point);
                data.intensities.push_back(intensity);
            }
        }

        return data;
    }

private:
    float deg2rad(float degrees) {
        return degrees * M_PI / 180.0f;
    }

    float generateNoise() {
        // Generate Gaussian noise
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::normal_distribution<float> dis(0.0f, noise_std_dev_);
        return dis(gen);
    }

    float calculateIntensity(const IntersectionResult& hit) {
        // Calculate return intensity based on:
        // - Material reflectivity
        // - Incident angle
        // - Distance (inverse square law)
        // - Surface roughness

        float material_reflectivity = hit.material.reflectivity;
        float incident_angle_factor = std::max(0.0f, hit.normal.dot(-hit.ray_direction));
        float distance_factor = 1.0f / (hit.distance * hit.distance + 1.0f);  // Avoid division by zero

        float intensity = material_reflectivity * incident_angle_factor * distance_factor;
        return std::min(1.0f, intensity);  // Clamp to [0, 1]
    }

    double getCurrentTime() {
        return std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now().time_since_epoch()
        ).count();
    }
};
```

### Multi-Beam LiDAR Simulation

Advanced LiDAR systems with multiple beams and complex configurations:

```cpp
class MultiBeamLiDARSimulator {
private:
    struct BeamConfig {
        float vertical_angle;  // Vertical angle of the beam
        float horizontal_angle; // Horizontal angle (for rotating scanners)
        float divergence;      // Beam divergence
    };

    std::vector<BeamConfig> beam_configs_;
    float rotation_speed_;     // RPM for spinning LiDAR
    float time_per_rotation_;  // Time for one full rotation
    int num_points_per_ring_;  // Points per horizontal ring

public:
    struct MultiBeamData {
        std::vector<std::vector<float>> range_data;  // [vertical_beams][horizontal_points]
        std::vector<std::vector<Eigen::Vector3f>> point_cloud;
        std::vector<std::vector<float>> intensities;
        std::vector<double> timestamps;  // Timestamp for each horizontal scan
    };

    MultiBeamData simulateMultiBeam(const Scene& scene,
                                   const Transform& lidar_pose,
                                   double start_time,
                                   double duration) {
        MultiBeamData data;

        int num_rotations = static_cast<int>(duration / time_per_rotation_);
        double time_step = time_per_rotation_ / num_points_per_ring_;

        for (int rot = 0; rot < num_rotations; ++rot) {
            std::vector<float> rotation_ranges;
            std::vector<Eigen::Vector3f> rotation_points;
            std::vector<float> rotation_intensities;

            for (int point_idx = 0; point_idx < num_points_per_ring_; ++point_idx) {
                double rotation_angle = (2.0 * M_PI * point_idx) / num_points_per_ring_;

                for (const auto& beam : beam_configs_) {
                    // Calculate beam direction accounting for rotation
                    float total_h_angle = rotation_angle + beam.horizontal_angle;
                    float h_cos = cos(total_h_angle);
                    float h_sin = sin(total_h_angle);
                    float v_cos = cos(beam.vertical_angle);
                    float v_sin = sin(beam.vertical_angle);

                    Eigen::Vector3f beam_dir_local(
                        v_cos * h_cos,
                        v_cos * h_sin,
                        v_sin
                    );

                    // Transform to world frame
                    Eigen::Vector3f beam_dir_world = lidar_pose.rotation * beam_dir_local;
                    Eigen::Vector3f beam_origin = lidar_pose.translation;

                    // Ray intersection
                    IntersectionResult hit = scene.rayIntersect(beam_origin, beam_dir_world);

                    float range = (hit.valid && hit.distance <= max_range_) ? hit.distance : max_range_;
                    range += generateNoise();

                    rotation_ranges.push_back(range);
                    rotation_points.push_back(hit.valid ? hit.point : Eigen::Vector3f(0,0,0));
                    rotation_intensities.push_back(hit.valid ? calculateIntensity(hit) : 0.0f);
                }
            }

            data.range_data.push_back(rotation_ranges);
            data.point_cloud.push_back(rotation_points);
            data.intensities.push_back(rotation_intensities);
            data.timestamps.push_back(start_time + rot * time_per_rotation_);
        }

        return data;
    }
};
```

## IMU Simulation

### Six-Axis IMU Model

IMU simulation includes accelerometer, gyroscope, and magnetometer models:

```cpp
class IMUSimulator {
private:
    // True values (noise-free)
    Eigen::Vector3f true_linear_acceleration_;
    Eigen::Vector3f true_angular_velocity_;
    Eigen::Vector3f true_magnetic_field_;

    // Noise parameters
    float accel_noise_density_;      // m/s²/√Hz
    float gyro_noise_density_;       // rad/s/√Hz
    float mag_noise_density_;        // Tesla/√Hz

    float accel_random_walk_;        // m/s²²/√Hz
    float gyro_random_walk_;         // rad/s²/√Hz
    float mag_random_walk_;          // Tesla/s/√Hz

    // Bias parameters
    Eigen::Vector3f accel_bias_;
    Eigen::Vector3f gyro_bias_;
    Eigen::Vector3f mag_bias_;

    // Scale factor errors
    Eigen::Vector3f accel_scale_factor_error_;
    Eigen::Vector3f gyro_scale_factor_error_;
    Eigen::Vector3f mag_scale_factor_error_;

    // Cross-coupling errors
    Eigen::Matrix3f accel_cross_coupling_;
    Eigen::Matrix3f gyro_cross_coupling_;
    Eigen::Matrix3f mag_cross_coupling_;

    double sampling_rate_;
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<float> noise_dist_;

public:
    struct IMUData {
        Eigen::Vector3f linear_acceleration;  // m/s²
        Eigen::Vector3f angular_velocity;     // rad/s
        Eigen::Vector3f magnetic_field;       // Tesla
        Eigen::Quaternionf orientation;       // Estimated from integration
        double timestamp;
    };

    IMUSimulator(double sampling_rate = 100.0)
        : sampling_rate_(sampling_rate), gen_(rd_()),
          noise_dist_(0.0, 1.0) {

        // Initialize with typical MEMS IMU parameters
        accel_noise_density_ = 0.004;   // mg/√Hz
        gyro_noise_density_ = 0.00014;  // °/s/√Hz
        mag_noise_density_ = 50e-9;     // Tesla/√Hz

        accel_random_walk_ = 0.0006;    // m/s²²/√Hz
        gyro_random_walk_ = 0.000012;   // rad/s²/√Hz
        mag_random_walk_ = 10e-9;       // Tesla/s/√Hz

        // Initialize biases (will drift over time)
        std::uniform_real_distribution<float> bias_dist(-0.01, 0.01);
        accel_bias_ = Eigen::Vector3f(bias_dist(gen_), bias_dist(gen_), bias_dist(gen_));
        gyro_bias_ = Eigen::Vector3f(bias_dist(gen_), bias_dist(gen_), bias_dist(gen_));
        mag_bias_ = Eigen::Vector3f(bias_dist(gen_), bias_dist(gen_), bias_dist(gen_));

        // Initialize scale factor errors
        accel_scale_factor_error_ = Eigen::Vector3f(0.001, 0.001, 0.001);
        gyro_scale_factor_error_ = Eigen::Vector3f(0.0001, 0.0001, 0.0001);
        mag_scale_factor_error_ = Eigen::Vector3f(0.0001, 0.0001, 0.0001);

        // Initialize cross-coupling matrices as identity + small errors
        accel_cross_coupling_ = Eigen::Matrix3f::Identity();
        gyro_cross_coupling_ = Eigen::Matrix3f::Identity();
        mag_cross_coupling_ = Eigen::Matrix3f::Identity();
    }

    IMUData simulateReading(const RobotState& state, double current_time) {
        IMUData data;
        data.timestamp = current_time;

        // Calculate true values from robot state
        calculateTrueValues(state, current_time);

        // Generate noise samples
        Eigen::Vector3f accel_noise = generateWhiteNoise(accel_noise_density_);
        Eigen::Vector3f gyro_noise = generateWhiteNoise(gyro_noise_density_);
        Eigen::Vector3f mag_noise = generateWhiteNoise(mag_noise_density_);

        // Generate random walk components
        static Eigen::Vector3f accel_bias_walk = Eigen::Vector3f::Zero();
        static Eigen::Vector3f gyro_bias_walk = Eigen::Vector3f::Zero();
        static Eigen::Vector3f mag_bias_walk = Eigen::Vector3f::Zero();

        double dt = 1.0 / sampling_rate_;
        accel_bias_walk += generateRandomWalk(accel_random_walk_, dt);
        gyro_bias_walk += generateRandomWalk(gyro_random_walk_, dt);
        mag_bias_walk += generateRandomWalk(mag_random_walk_, dt);

        // Apply sensor model
        data.linear_acceleration = applyAccelerometerModel(true_linear_acceleration_,
                                                          accel_noise,
                                                          accel_bias_walk + accel_bias_,
                                                          dt);
        data.angular_velocity = applyGyroscopeModel(true_angular_velocity_,
                                                   gyro_noise,
                                                   gyro_bias_walk + gyro_bias_,
                                                   dt);
        data.magnetic_field = applyMagnetometerModel(true_magnetic_field_,
                                                    mag_noise,
                                                    mag_bias_walk + mag_bias_,
                                                    dt);

        return data;
    }

private:
    void calculateTrueValues(const RobotState& state, double current_time) {
        // Calculate true linear acceleration in IMU frame
        // This would involve transforming robot acceleration to IMU frame
        Eigen::Vector3f robot_acceleration = state.acceleration;
        Eigen::Vector3f robot_angular_velocity = state.angular_velocity;
        Eigen::Vector3f gravity(0, 0, -9.81);

        // Transform robot acceleration to IMU frame
        // Add gravity component (IMU measures specific force, not gravitational acceleration)
        true_linear_acceleration_ = state.imu_rotation.transpose() * (robot_acceleration - gravity);

        // Calculate true angular velocity
        true_angular_velocity_ = state.imu_rotation.transpose() * robot_angular_velocity;

        // Calculate true magnetic field (simplified - assumes constant field)
        true_magnetic_field_ = Eigen::Vector3f(0.22, 0.0, 0.45);  // Approximate Earth's magnetic field
    }

    Eigen::Vector3f generateWhiteNoise(float noise_density) {
        double dt_sqrt = sqrt(1.0 / sampling_rate_);
        return Eigen::Vector3f(
            noise_dist_(gen_) * noise_density * dt_sqrt,
            noise_dist_(gen_) * noise_density * dt_sqrt,
            noise_dist_(gen_) * noise_density * dt_sqrt
        );
    }

    Eigen::Vector3f generateRandomWalk(float random_walk, double dt) {
        double dt_sqrt = sqrt(dt);
        return Eigen::Vector3f(
            noise_dist_(gen_) * random_walk * dt_sqrt,
            noise_dist_(gen_) * random_walk * dt_sqrt,
            noise_dist_(gen_) * random_walk * dt_sqrt
        );
    }

    Eigen::Vector3f applyAccelerometerModel(const Eigen::Vector3f& true_accel,
                                           const Eigen::Vector3f& noise,
                                           const Eigen::Vector3f& bias,
                                           double dt) {
        // Apply scale factor errors
        Eigen::Vector3f scaled_accel = true_accel.cwiseProduct(Eigen::Vector3f::Ones() + accel_scale_factor_error_);

        // Apply cross-coupling
        scaled_accel = accel_cross_coupling_ * scaled_accel;

        // Add bias and noise
        Eigen::Vector3f noisy_accel = scaled_accel + bias + noise;

        return noisy_accel;
    }

    Eigen::Vector3f applyGyroscopeModel(const Eigen::Vector3f& true_gyro,
                                       const Eigen::Vector3f& noise,
                                       const Eigen::Vector3f& bias,
                                       double dt) {
        // Apply scale factor errors
        Eigen::Vector3f scaled_gyro = true_gyro.cwiseProduct(Eigen::Vector3f::Ones() + gyro_scale_factor_error_);

        // Apply cross-coupling
        scaled_gyro = gyro_cross_coupling_ * scaled_gyro;

        // Add bias and noise
        Eigen::Vector3f noisy_gyro = scaled_gyro + bias + noise;

        return noisy_gyro;
    }

    Eigen::Vector3f applyMagnetometerModel(const Eigen::Vector3f& true_mag,
                                          const Eigen::Vector3f& noise,
                                          const Eigen::Vector3f& bias,
                                          double dt) {
        // Apply scale factor errors
        Eigen::Vector3f scaled_mag = true_mag.cwiseProduct(Eigen::Vector3f::Ones() + mag_scale_factor_error_);

        // Apply cross-coupling
        scaled_mag = mag_cross_coupling_ * scaled_mag;

        // Add bias and noise
        Eigen::Vector3f noisy_mag = scaled_mag + bias + noise;

        return noisy_mag;
    }
};
```

## GPS Simulation

### GNSS Receiver Model

GPS simulation models satellite signal reception and positioning errors:

```cpp
class GPSSimulator {
private:
    // Satellite constellation simulation
    struct Satellite {
        int prn;                           // Pseudo-random noise code
        Eigen::Vector3f position;         // Position in ECEF coordinates
        double clock_bias;                 // Clock bias in seconds
        bool healthy;                      // Signal health status
    };

    std::vector<Satellite> satellites_;
    Eigen::Vector3f true_position_ecef_;  // True position in ECEF
    double true_time_;                    // True GPS time

    // Error models
    double ephemeris_error_std_;          // Satellite position error (meters)
    double ionospheric_delay_std_;        // Ionospheric delay error (meters)
    double tropospheric_delay_std_;       // Tropospheric delay error (meters)
    double multipath_error_std_;          // Multipath error (meters)
    double receiver_noise_std_;           // Receiver noise (meters)

    // Environmental factors
    std::vector<Eigen::Vector3f> obstructions_;  // Buildings, trees, etc.
    double urban_canyon_factor_;          // Urban canyon multipath multiplier

public:
    struct GPSData {
        Eigen::Vector3f position_ecef;    // Position in ECEF coordinates
        Eigen::Vector3f position_llh;     // Position in Lat/Long/Height
        float horizontal_accuracy;         // Horizontal accuracy (meters)
        float vertical_accuracy;           // Vertical accuracy (meters)
        int num_satellites;               // Number of satellites used
        double timestamp;
        bool valid_fix;                   // Whether fix is valid
    };

    GPSSimulator() {
        initializeSatellites();
        ephemeris_error_std_ = 2.0;        // meters
        ionospheric_delay_std_ = 5.0;      // meters
        tropospheric_delay_std_ = 2.5;     // meters
        multipath_error_std_ = 3.0;        // meters
        receiver_noise_std_ = 1.0;         // meters
        urban_canyon_factor_ = 2.0;        // Multipath multiplier in urban areas
    }

    GPSData simulateReading(const RobotState& state, double current_time) {
        GPSData data;
        data.timestamp = current_time;

        // Convert robot position to ECEF coordinates
        true_position_ecef_ = llhToECEF(state.latitude, state.longitude, state.altitude);
        true_time_ = current_time;

        // Determine visible satellites
        auto visible_satellites = getVisibleSatellites(true_position_ecef_);

        if (visible_satellites.size() < 4) {
            data.valid_fix = false;
            data.num_satellites = visible_satellites.size();
            return data;
        }

        // Calculate pseudoranges with errors
        std::vector<PseudorangeMeasurement> measurements;
        for (const auto& sat : visible_satellites) {
            double pseudorange = calculatePseudorange(sat);
            double error = generateGPSNoise(sat, state.position);
            pseudorange += error;

            measurements.push_back({
                sat.prn,
                pseudorange,
                calculateAzimuthElevation(sat),
                calculateSignalStrength(sat, state.position)
            });
        }

        // Perform position calculation using least squares
        auto [position, accuracy] = calculatePosition(measurements);

        data.position_ecef = position;
        data.position_llh = ecefToLLH(position);
        data.horizontal_accuracy = accuracy.first;
        data.vertical_accuracy = accuracy.second;
        data.num_satellites = measurements.size();
        data.valid_fix = true;

        return data;
    }

private:
    struct PseudorangeMeasurement {
        int prn;
        double pseudorange;
        std::pair<double, double> az_el;  // Azimuth, elevation
        double signal_strength;
    };

    void initializeSatellites() {
        // Initialize GPS constellation (simplified model)
        // In reality, this would use precise ephemeris data
        for (int i = 1; i <= 32; ++i) {  // GPS has 32 operational satellites
            Satellite sat;
            sat.prn = i;

            // Simplified orbital mechanics to place satellites
            double orbit_radius = 26560000.0;  // GPS orbit radius in meters
            double angle = (2.0 * M_PI * i) / 32.0;

            sat.position = Eigen::Vector3f(
                orbit_radius * cos(angle),
                orbit_radius * sin(angle),
                0  // Simplified circular orbit in XY plane
            );

            sat.clock_bias = 0.0;  // Simplified
            sat.healthy = true;

            satellites_.push_back(sat);
        }
    }

    std::vector<Satellite> getVisibleSatellites(const Eigen::Vector3f& receiver_pos) {
        std::vector<Satellite> visible;

        for (auto& sat : satellites_) {
            Eigen::Vector3f sat_to_rec = receiver_pos - sat.position;
            double elevation = calculateElevationAngle(sat_to_rec);

            // Only satellites above horizon (positive elevation) are visible
            if (elevation > 5.0 * M_PI / 180.0) {  // 5 degree mask
                // Check for obstructions
                if (!isObstructed(sat.position, receiver_pos)) {
                    visible.push_back(sat);
                }
            }
        }

        return visible;
    }

    bool isObstructed(const Eigen::Vector3f& sat_pos, const Eigen::Vector3f& rec_pos) {
        // Check if satellite signal path is obstructed by buildings/terrain
        Eigen::Vector3f direction = (sat_pos - rec_pos).normalized();

        for (const auto& obstruction : obstructions_) {
            // Simplified collision detection
            // In reality, this would be more sophisticated
            Eigen::Vector3f to_obstruction = obstruction - rec_pos;
            double projection = to_obstruction.dot(direction);

            if (projection > 0) {  // Obstruction is in front of receiver
                Eigen::Vector3f closest_point = rec_pos + projection * direction;
                double distance = (closest_point - obstruction).norm();

                if (distance < 5.0) {  // 5 meter clearance threshold
                    return true;
                }
            }
        }

        return false;
    }

    double calculatePseudorange(const Satellite& sat) {
        // Calculate geometric range + satellite clock bias + atmospheric delays
        double geometric_range = (sat.position - true_position_ecef_).norm();

        // Add satellite clock bias
        double range_with_clock = geometric_range + sat.clock_bias * 299792458.0;  // Speed of light

        // Add atmospheric delays
        double iono_delay = generateAtmosphericDelay(ionospheric_delay_std_);
        double tropo_delay = generateAtmosphericDelay(tropospheric_delay_std_);

        return range_with_clock + iono_delay + tropo_delay;
    }

    double generateAtmosphericDelay(double std_dev) {
        // Generate atmospheric delay error
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::normal_distribution<double> dist(0.0, std_dev);
        return dist(gen);
    }

    std::pair<double, double> calculateAzimuthElevation(const Satellite& sat) {
        // Calculate azimuth and elevation angles
        Eigen::Vector3f sat_vec = sat.position - true_position_ecef_;

        // Convert to local tangent plane (ENU coordinates)
        auto [lat, lon, alt] = ecefToLLHComponents(true_position_ecef_);
        Eigen::Vector3f enu = ecefToENU(sat_vec, lat, lon);

        double azimuth = atan2(enu.x(), enu.y());  // East of North
        double elevation = atan2(enu.z(), sqrt(enu.x()*enu.x() + enu.y()*enu.y()));

        return {azimuth, elevation};
    }

    double calculateSignalStrength(const Satellite& sat, const Eigen::Vector3f& pos) {
        // Calculate received signal strength based on distance and antenna pattern
        double distance = (sat.position - pos).norm();

        // Free space path loss
        double fspl = 20 * log10(distance) + 20 * log10(1575.42e6) - 147.55;  // L1 frequency
        double received_power = -130 - fspl;  // -130 dBm is typical GPS signal strength

        return std::max(-160.0, received_power);  // Minimum detectable signal
    }

    double generateGPSNoise(const Satellite& sat, const Eigen::Vector3f& pos) {
        double total_error = 0.0;

        // Ephemeris error
        total_error += generateNormalNoise(ephemeris_error_std_);

        // Ionospheric delay (correlated with elevation angle)
        auto [az, el] = calculateAzimuthElevation(sat);
        double iono_factor = 1.0 / sin(el + 0.1);  // Slant factor
        total_error += generateNormalNoise(ionospheric_delay_std_) * iono_factor;

        // Tropospheric delay
        double tropo_factor = 1.0 / sin(el + 0.1);  // Also elevation dependent
        total_error += generateNormalNoise(tropospheric_delay_std_) * tropo_factor;

        // Multipath (depends on environment)
        double multipath_factor = isUrbanEnvironment(pos) ? urban_canyon_factor_ : 1.0;
        total_error += generateNormalNoise(multipath_error_std_) * multipath_factor;

        // Receiver noise
        total_error += generateNormalNoise(receiver_noise_std_);

        return total_error;
    }

    double generateNormalNoise(double std_dev) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::normal_distribution<double> dist(0.0, std_dev);
        return dist(gen);
    }

    bool isUrbanEnvironment(const Eigen::Vector3f& pos) {
        // Simplified urban detection
        // In reality, this would check against map data
        return false;  // Simplified assumption
    }

    std::pair<Eigen::Vector3f, std::pair<float, float>> calculatePosition(
        const std::vector<PseudorangeMeasurement>& measurements) {

        // Simplified position calculation using least squares
        // In reality, this would be more sophisticated with iterative methods

        if (measurements.size() < 4) {
            return {true_position_ecef_, {100.0f, 100.0f}};  // Poor accuracy
        }

        // For simplicity, return position with some error
        double pos_error = generateNormalNoise(3.0);  // Typical GPS accuracy
        double alt_error = generateNormalNoise(5.0);  // Typically worse vertical accuracy

        Eigen::Vector3f estimated_pos = true_position_ecef_ + Eigen::Vector3f(
            generateNormalNoise(3.0),
            generateNormalNoise(3.0),
            generateNormalNoise(5.0)
        );

        return {estimated_pos, {3.0f, 5.0f}};
    }

    Eigen::Vector3f llhToECEF(double lat, double lon, double alt) {
        // Convert Latitude/Longitude/Height to Earth-Centered Earth-Fixed
        const double a = 6378137.0;  // Semi-major axis
        const double f = 1.0/298.257223563;  // Flattening
        const double e2 = 2*f - f*f;  // First eccentricity squared

        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;

        double N = a / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));

        double x = (N + alt) * cos(lat_rad) * cos(lon_rad);
        double y = (N + alt) * cos(lat_rad) * sin(lon_rad);
        double z = (N * (1 - e2) + alt) * sin(lat_rad);

        return Eigen::Vector3f(x, y, z);
    }

    Eigen::Vector3f ecefToLLH(const Eigen::Vector3f& ecef) {
        // Convert ECEF to Latitude/Longitude/Height (simplified)
        const double a = 6378137.0;
        const double f = 1.0/298.257223563;
        const double e2 = 2*f - f*f;

        double x = ecef.x();
        double y = ecef.y();
        double z = ecef.z();

        double p = sqrt(x*x + y*y);
        double theta = atan2(z*a, p*(1-e2));

        double lat = atan2(z + e2*a*pow(sin(theta), 3), p - e2*a*pow(cos(theta), 3));
        double lon = atan2(y, x);
        double N = a / sqrt(1 - e2*sin(lat)*sin(lat));
        double alt = p/cos(lat) - N;

        return Eigen::Vector3f(lat*180.0/M_PI, lon*180.0/M_PI, alt);
    }

    std::tuple<double, double, double> ecefToLLHComponents(const Eigen::Vector3f& ecef) {
        auto llh = ecefToLLH(ecef);
        return std::make_tuple(llh.x(), llh.y(), llh.z());
    }

    Eigen::Vector3f ecefToENU(const Eigen::Vector3f& ecef_vec, double lat, double lon) {
        // Convert ECEF vector to East-North-Up local coordinates
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;

        Eigen::Matrix3f rotation_matrix;
        rotation_matrix << -sin(lon_rad),           cos(lon_rad),          0,
                          -sin(lat_rad)*cos(lon_rad), -sin(lat_rad)*sin(lon_rad), cos(lat_rad),
                           cos(lat_rad)*cos(lon_rad),  cos(lat_rad)*sin(lon_rad), sin(lat_rad);

        return rotation_matrix * ecef_vec;
    }
};
```

## Sensor Fusion Simulation

### Kalman Filter Integration

Combining multiple simulated sensors using Kalman filtering:

```cpp
class SensorFusionSimulator {
private:
    // Individual sensor simulators
    IMUSimulator imu_sim_;
    GPSSimulator gps_sim_;
    LiDARSimulator lidar_sim_;
    CameraSimulator camera_sim_;

    // Kalman filter state
    Eigen::VectorXd state_vector_;  // [px, py, pz, vx, vy, vz, qx, qy, qz, qw]
    Eigen::MatrixXd covariance_matrix_;
    Eigen::MatrixXd process_noise_;
    Eigen::MatrixXd measurement_noise_;

    double last_update_time_;

public:
    struct FusedState {
        Eigen::Vector3f position;
        Eigen::Vector3f velocity;
        Eigen::Quaternionf orientation;
        Eigen::Vector3f angular_velocity;
        Eigen::Vector3f linear_acceleration;
        Eigen::Matrix<double, 6, 6> position_covariance;
        double timestamp;
    };

    SensorFusionSimulator() {
        initializeFilter();
    }

    FusedState simulateFusedState(const RobotState& true_state, double current_time) {
        FusedState fused_state;
        fused_state.timestamp = current_time;

        // Simulate individual sensor readings
        auto imu_data = imu_sim_.simulateReading(true_state, current_time);
        auto gps_data = gps_sim_.simulateReading(true_state, current_time);
        // LiDAR and camera data would be simulated similarly

        // Prediction step (predict state forward in time)
        double dt = current_time - last_update_time_;
        predictState(dt);

        // Update step based on available measurements
        if (gps_data.valid_fix && (current_time - last_gps_update_) > 1.0) {  // 1 Hz GPS updates
            updateWithGPS(gps_data);
            last_gps_update_ = current_time;
        }

        if (dt > 0.01) {  // 100 Hz IMU updates
            updateWithIMU(imu_data);
        }

        // Extract fused state from filter
        extractState(fused_state);

        last_update_time_ = current_time;

        return fused_state;
    }

private:
    void initializeFilter() {
        // Initialize 16-state Extended Kalman Filter
        // [position, velocity, orientation quaternion, biases]
        state_vector_ = Eigen::VectorXd::Zero(16);  // [px, py, pz, vx, vy, vz, qw, qx, qy, qz, accel_bias, gyro_bias]
        covariance_matrix_ = Eigen::MatrixXd::Identity(16, 16) * 100.0;

        // Process noise (motion model uncertainty)
        process_noise_ = Eigen::MatrixXd::Identity(16, 16);
        process_noise_.topLeftCorner(3, 3) *= 0.1;    // Position process noise
        process_noise_.block(3, 3, 3, 3) *= 1.0;      // Velocity process noise
        process_noise_.block(6, 6, 4, 4) *= 0.01;     // Orientation process noise
        process_noise_.block(10, 10, 3, 3) *= 0.001;  // Accelerometer bias process noise
        process_noise_.block(13, 13, 3, 3) *= 0.0001; // Gyroscope bias process noise

        last_update_time_ = 0.0;
        last_gps_update_ = 0.0;
    }

    void predictState(double dt) {
        // Predict state forward using motion model
        // This is a simplified example - in reality, this would be more complex

        // Extract current state
        Eigen::Vector3d pos = state_vector_.segment(0, 3);
        Eigen::Vector3d vel = state_vector_.segment(3, 3);
        Eigen::Quaterniond orient(state_vector_.segment(6, 4).normalized());
        Eigen::Vector3d accel_bias = state_vector_.segment(10, 3);
        Eigen::Vector3d gyro_bias = state_vector_.segment(13, 3);

        // Simple constant velocity model with gravity compensation
        Eigen::Vector3d gravity(0, 0, 9.81);
        Eigen::Vector3d specific_force = orient.toRotationMatrix() * (vel / dt) - gravity;

        // Update state predictions
        Eigen::Vector3d new_pos = pos + vel * dt + 0.5 * specific_force * dt * dt;
        Eigen::Vector3d new_vel = vel + specific_force * dt;

        // Update state vector
        state_vector_.segment(0, 3) = new_pos;
        state_vector_.segment(3, 3) = new_vel;

        // Linearize motion model and propagate covariance
        Eigen::MatrixXd F = computeStateTransitionMatrix(dt);
        covariance_matrix_ = F * covariance_matrix_ * F.transpose() + process_noise_ * dt;
    }

    Eigen::MatrixXd computeStateTransitionMatrix(double dt) {
        // Compute state transition matrix F for linearized system
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(16, 16);

        // Position-velocity coupling
        F.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * dt;

        // Velocity-acceleration coupling (simplified)
        F.block(3, 6, 3, 3) = Eigen::Matrix3d::Identity() * dt;

        return F;
    }

    void updateWithGPS(const GPSSimulator::GPSData& gps_data) {
        // Measurement model: h(x) = [px, py, pz] (position measurement)
        Eigen::VectorXd measurement(3);
        measurement << gps_data.position_ecef.x(),
                       gps_data.position_ecef.y(),
                       gps_data.position_ecef.z();

        // Measurement matrix H (maps state to measurement)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 16);
        H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();  // Position measurements

        // Measurement noise covariance
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
        R(0, 0) = gps_data.horizontal_accuracy * gps_data.horizontal_accuracy;
        R(1, 1) = gps_data.horizontal_accuracy * gps_data.horizontal_accuracy;
        R(2, 2) = gps_data.vertical_accuracy * gps_data.vertical_accuracy;

        // Kalman update equations
        Eigen::MatrixXd S = H * covariance_matrix_ * H.transpose() + R;  // Innovation covariance
        Eigen::MatrixXd K = covariance_matrix_ * H.transpose() * S.inverse();  // Kalman gain

        // Innovation (measurement residual)
        Eigen::VectorXd y = measurement - H * state_vector_;

        // Update state and covariance
        state_vector_ = state_vector_ + K * y;
        covariance_matrix_ = (Eigen::MatrixXd::Identity(16, 16) - K * H) * covariance_matrix_;
    }

    void updateWithIMU(const IMUSimulator::IMUData& imu_data) {
        // For IMU, we typically use it in the prediction step rather than direct update
        // since IMU provides relative measurements (deltas)
        // This would involve more complex integration in practice
    }

    void extractState(FusedState& state) {
        // Extract position, velocity, and orientation from state vector
        state.position = state_vector_.segment(0, 3).cast<float>();
        state.velocity = state_vector_.segment(3, 3).cast<float>();
        Eigen::Vector4d quat_vec = state_vector_.segment(6, 4).normalized();
        state.orientation = Eigen::Quaternionf(quat_vec.w(), quat_vec.x(), quat_vec.y(), quat_vec.z());

        // Extract position covariance
        state.position_covariance = covariance_matrix_.block(0, 0, 6, 6);
    }

    double last_gps_update_;
};
```

## Integration with Simulation Environments

### Gazebo Sensor Plugins

Integrating sensor simulation with physics engines:

```xml
<!-- Example Gazebo sensor plugin configuration -->
<sdf version="1.7">
  <model name="sensor_equipped_robot">
    <link name="base_link">
      <!-- IMU Sensor -->
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0014</stddev>
                <bias_mean>0.0</bias_mean>
                <bias_stddev>0.0</bias_stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0014</stddev>
                <bias_mean>0.0</bias_mean>
                <bias_stddev>0.0</bias_stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0014</stddev>
                <bias_mean>0.0</bias_mean>
                <bias_stddev>0.0</bias_stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.017</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.017</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.017</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>

      <!-- Camera Sensor -->
      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <frame_name>camera_frame</frame_name>
          <min_depth>0.1</min_depth>
          <max_depth>10.0</max_depth>
        </plugin>
      </sensor>

      <!-- LiDAR Sensor -->
      <sensor name="lidar" type="ray">
        <ray>
          <scan>
            <horizontal>
              <samples>720</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>30</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
          <topicName>/laser_scan</topicName>
          <frameName>lidar_frame</frameName>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
```

### ROS Integration

Sensor simulation integration with ROS:

```cpp
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

class ROSSensorSimulator {
private:
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher lidar_pub_;
    ros::Publisher camera_pub_;

    IMUSimulator imu_sim_;
    GPSSimulator gps_sim_;
    LiDARSimulator lidar_sim_;
    CameraSimulator camera_sim_;

public:
    ROSSensorSimulator() {
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data", 10);
        gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);
        lidar_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 10);
        camera_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/image_raw", 10);

        // Timer for sensor updates
        ros::Timer timer = nh_.createTimer(ros::Duration(0.01), &ROSSensorSimulator::updateSensors, this);
    }

    void updateSensors(const ros::TimerEvent&) {
        // Get current robot state (from TF, odometry, or other sources)
        RobotState robot_state = getCurrentRobotState();

        // Simulate and publish IMU data
        auto imu_data = imu_sim_.simulateReading(robot_state, ros::Time::now().toSec());
        publishIMU(imu_data);

        // Simulate and publish GPS data
        auto gps_data = gps_sim_.simulateReading(robot_state, ros::Time::now().toSec());
        publishGPS(gps_data);

        // Simulate and publish LiDAR data
        auto lidar_data = lidar_sim_.simulateScan(getCurrentScene(), getCurrentTransform());
        publishLiDAR(lidar_data);

        // Simulate and publish camera data
        auto camera_image = camera_sim_.simulateImage(getCurrentScene(), getCurrentTransform());
        publishCamera(camera_image);
    }

private:
    RobotState getCurrentRobotState() {
        // Implementation would get current robot state from TF or other sources
        RobotState state;
        // ... populate state from ROS topics/TF
        return state;
    }

    Scene getCurrentScene() {
        // Implementation would get current scene/environment
        Scene scene;
        // ... populate scene from environment description
        return scene;
    }

    Transform getCurrentTransform() {
        // Implementation would get current sensor transform
        Transform transform;
        // ... populate transform
        return transform;
    }

    void publishIMU(const IMUSimulator::IMUData& data) {
        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time(data.timestamp);
        msg.header.frame_id = "imu_link";

        msg.linear_acceleration.x = data.linear_acceleration.x();
        msg.linear_acceleration.y = data.linear_acceleration.y();
        msg.linear_acceleration.z = data.linear_acceleration.z();

        msg.angular_velocity.x = data.angular_velocity.x();
        msg.angular_velocity.y = data.angular_velocity.y();
        msg.angular_velocity.z = data.angular_velocity.z();

        // Orientation would be integrated from gyro data
        // For simplicity, we'll leave it as zero

        // Set covariance matrices based on noise characteristics
        for (int i = 0; i < 9; ++i) {
            msg.linear_acceleration_covariance[i] = i % 4 == 0 ? 0.0001 : 0;  // Diagonal elements
            msg.angular_velocity_covariance[i] = i % 4 == 0 ? 0.00002 : 0;   // Diagonal elements
        }

        imu_pub_.publish(msg);
    }

    void publishGPS(const GPSSimulator::GPSData& data) {
        sensor_msgs::NavSatFix msg;
        msg.header.stamp = ros::Time(data.timestamp);
        msg.header.frame_id = "gps_link";

        msg.latitude = data.position_llh.x();
        msg.longitude = data.position_llh.y();
        msg.altitude = data.position_llh.z();

        // Set status and covariance based on fix quality
        msg.status.status = data.valid_fix ?
            sensor_msgs::NavSatStatus::STATUS_FIX :
            sensor_msgs::NavSatStatus::STATUS_NO_FIX;

        msg.position_covariance[0] = data.horizontal_accuracy * data.horizontal_accuracy;  // XX
        msg.position_covariance[4] = data.horizontal_accuracy * data.horizontal_accuracy;  // YY
        msg.position_covariance[8] = data.vertical_accuracy * data.vertical_accuracy;      // ZZ

        msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        gps_pub_.publish(msg);
    }

    void publishLiDAR(const LiDARSimulator::LiDARData& data) {
        sensor_msgs::LaserScan msg;
        msg.header.stamp = ros::Time(data.timestamp);
        msg.header.frame_id = "lidar_link";

        msg.angle_min = -M_PI;
        msg.angle_max = M_PI;
        msg.angle_increment = (2.0 * M_PI) / data.ranges.size();
        msg.time_increment = 0.0;  // Not applicable for simulated data
        msg.scan_time = 0.1;       // 10Hz for example
        msg.range_min = 0.1;
        msg.range_max = 30.0;

        msg.ranges.resize(data.ranges.size());
        for (size_t i = 0; i < data.ranges.size(); ++i) {
            msg.ranges[i] = data.ranges[i];
        }

        lidar_pub_.publish(msg);
    }

    void publishCamera(const cv::Mat& image) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "camera_link";

        camera_pub_.publish(msg);
    }
};
```

## Performance Optimization

### Parallel Processing

Optimizing sensor simulation for real-time performance:

```cpp
#include <thread>
#include <future>
#include <queue>

class ParallelSensorSimulator {
private:
    std::vector<std::thread> worker_threads_;
    std::queue<std::future<SensorData>> result_queue_;
    std::mutex queue_mutex_;
    int num_workers_;

public:
    struct SensorData {
        enum Type { IMU, GPS, LIDAR, CAMERA, COUNT };
        Type type;
        double timestamp;
        std::vector<uint8_t> data;  // Serialized sensor data
    };

    std::vector<SensorData> simulateParallel(const Scene& scene,
                                           const RobotState& state,
                                           double current_time) {
        std::vector<std::future<SensorData>> futures;

        // Launch sensor simulations in parallel
        futures.push_back(std::async(std::launch::async, [this, scene, state, current_time]() {
            return simulateIMU(scene, state, current_time);
        }));

        futures.push_back(std::async(std::launch::async, [this, scene, state, current_time]() {
            return simulateGPS(scene, state, current_time);
        }));

        futures.push_back(std::async(std::launch::async, [this, scene, state, current_time]() {
            return simulateLiDAR(scene, state, current_time);
        }));

        futures.push_back(std::async(std::launch::async, [this, scene, state, current_time]() {
            return simulateCamera(scene, state, current_time);
        }));

        // Collect results
        std::vector<SensorData> results;
        for (auto& future : futures) {
            results.push_back(future.get());
        }

        return results;
    }

    // Sensor-specific simulation functions would be implemented here
    SensorData simulateIMU(const Scene& scene, const RobotState& state, double time) {
        // IMU simulation implementation
        IMUSimulator imu_sim;
        auto data = imu_sim.simulateReading(state, time);

        SensorData result;
        result.type = SensorData::IMU;
        result.timestamp = time;

        // Serialize IMU data
        std::vector<uint8_t> serialized_data(sizeof(data.linear_acceleration) +
                                           sizeof(data.angular_velocity) +
                                           sizeof(data.magnetic_field));

        // Copy data to serialized buffer
        size_t offset = 0;
        memcpy(serialized_data.data() + offset, &data.linear_acceleration, sizeof(data.linear_acceleration));
        offset += sizeof(data.linear_acceleration);
        memcpy(serialized_data.data() + offset, &data.angular_velocity, sizeof(data.angular_velocity));
        offset += sizeof(data.angular_velocity);
        memcpy(serialized_data.data() + offset, &data.magnetic_field, sizeof(data.magnetic_field));

        result.data = serialized_data;
        return result;
    }

    SensorData simulateGPS(const Scene& scene, const RobotState& state, double time) {
        // GPS simulation implementation
        GPSSimulator gps_sim;
        auto data = gps_sim.simulateReading(state, time);

        SensorData result;
        result.type = SensorData::GPS;
        result.timestamp = time;

        // Similar serialization for GPS data
        // ...
        return result;
    }

    SensorData simulateLiDAR(const Scene& scene, const RobotState& state, double time) {
        // LiDAR simulation implementation
        LiDARSimulator lidar_sim;
        auto data = lidar_sim.simulateScan(scene, state.getTransform());

        SensorData result;
        result.type = SensorData::LIDAR;
        result.timestamp = time;

        // Serialize LiDAR data
        // ...
        return result;
    }

    SensorData simulateCamera(const Scene& scene, const RobotState& state, double time) {
        // Camera simulation implementation
        CameraSimulator cam_sim;
        auto image = cam_sim.simulateImage(scene, state.getTransform());

        SensorData result;
        result.type = SensorData::CAMERA;
        result.timestamp = time;

        // Serialize camera image
        std::vector<uint8_t> encoded_image;
        cv::imencode(".jpg", image, encoded_image);
        result.data = encoded_image;

        return result;
    }
};
```

## Validation and Testing

### Sensor Model Validation

Validating sensor simulation accuracy:

```cpp
class SensorValidator {
public:
    struct ValidationResult {
        bool passed = false;
        double error_metric = 0.0;
        std::string error_description;
        std::vector<double> statistics;  // mean, std, min, max of errors
    };

    ValidationResult validateIMUModel(const std::vector<IMUData>& simulated_data,
                                     const std::vector<IMUData>& reference_data) {
        ValidationResult result;

        if (simulated_data.size() != reference_data.size()) {
            result.error_description = "Data size mismatch";
            return result;
        }

        std::vector<double> errors;
        for (size_t i = 0; i < simulated_data.size(); ++i) {
            double accel_error = (simulated_data[i].linear_acceleration -
                                 reference_data[i].linear_acceleration).norm();
            double gyro_error = (simulated_data[i].angular_velocity -
                                reference_data[i].angular_velocity).norm();

            errors.push_back(accel_error + gyro_error);
        }

        // Calculate statistics
        double mean_error = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
        double variance = 0.0;
        for (double error : errors) {
            variance += (error - mean_error) * (error - mean_error);
        }
        variance /= errors.size();
        double std_dev = sqrt(variance);

        result.statistics = {mean_error, std_dev,
                            *std::min_element(errors.begin(), errors.end()),
                            *std::max_element(errors.begin(), errors.end())};

        // Check if errors are within acceptable bounds
        result.passed = mean_error < 0.1;  // 0.1 m/s² for acceleration
        result.error_metric = mean_error;

        if (!result.passed) {
            result.error_description = "Mean error exceeds threshold: " + std::to_string(mean_error);
        }

        return result;
    }

    ValidationResult validateLiDARModel(const LiDARData& simulated,
                                       const LiDARData& reference) {
        ValidationResult result;

        if (simulated.ranges.size() != reference.ranges.size()) {
            result.error_description = "Range data size mismatch";
            return result;
        }

        std::vector<double> errors;
        for (size_t i = 0; i < simulated.ranges.size(); ++i) {
            double error = std::abs(simulated.ranges[i] - reference.ranges[i]);
            errors.push_back(error);
        }

        // Calculate statistics
        double mean_error = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
        result.error_metric = mean_error;

        // LiDAR typically has cm-level accuracy
        result.passed = mean_error < 0.05;  // 5 cm threshold
        result.error_description = mean_error < 0.05 ? "" :
            "Mean range error exceeds 5cm threshold: " + std::to_string(mean_error);

        return result;
    }

    ValidationResult validateCameraModel(const cv::Mat& simulated,
                                        const cv::Mat& reference) {
        ValidationResult result;

        if (simulated.size() != reference.size()) {
            result.error_description = "Image size mismatch";
            return result;
        }

        // Calculate structural similarity (SSIM) or other image quality metrics
        double ssim_value = calculateSSIM(simulated, reference);

        result.error_metric = 1.0 - ssim_value;  // Error = 1 - SSIM
        result.passed = ssim_value > 0.8;  // Threshold for visual similarity
        result.error_description = ssim_value > 0.8 ? "" :
            "SSIM below threshold: " + std::to_string(ssim_value);

        return result;
    }

private:
    double calculateSSIM(const cv::Mat& img1, const cv::Mat& img2) {
        // Simplified SSIM calculation
        // In practice, this would use a more sophisticated implementation

        cv::Mat gray1, gray2;
        cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);

        // Calculate mean and variance
        cv::Scalar mean1, mean2, var1, var2, covar;
        cv::meanStdDev(gray1, mean1, var1);
        cv::meanStdDev(gray2, mean2, var2);

        // Simplified SSIM formula
        double mu1 = mean1[0];
        double mu2 = mean2[0];
        double sigma1_sq = var1[0] * var1[0];
        double sigma2_sq = var2[0] * var2[0];

        double c1 = (0.01 * 255) * (0.01 * 255);
        double c2 = (0.03 * 255) * (0.03 * 255);

        double numerator = (2 * mu1 * mu2 + c1) * (2 * sqrt(sigma1_sq) * sqrt(sigma2_sq) + c2);
        double denominator = (mu1*mu1 + mu2*mu2 + c1) * (sigma1_sq + sigma2_sq + c2);

        return numerator / denominator;
    }
};
```

## Common Challenges and Solutions

### The Reality Gap

One of the biggest challenges in sensor simulation is the reality gap - the difference between simulated and real sensor behavior:

**Solutions:**
- **Domain Randomization:** Randomize environment parameters during training
- **System Identification:** Calibrate simulation parameters using real data
- **Adversarial Training:** Use adversarial networks to make sim data more realistic

### Computational Performance

Sensor simulation can be computationally expensive:

**Optimizations:**
- **Level of Detail (LOD):** Reduce simulation quality for distant objects
- **Culling:** Skip simulation for sensors facing away from interesting areas
- **Temporal Coherence:** Reuse previous frame calculations where possible

### Sensor-Specific Challenges

Each sensor type has unique challenges:

**Cameras:**
- Lighting condition variations
- Lens distortion modeling
- Dynamic range limitations

**LiDAR:**
- Multi-path reflections
- Variable return intensities
- Weather effects (fog, rain)

**IMU:**
- Temperature effects
- Vibration sensitivity
- Calibration drift

## Best Practices

### Model Fidelity vs. Performance

Balance simulation accuracy with computational requirements:

1. **Start Simple:** Begin with basic models and add complexity gradually
2. **Validate Incrementally:** Test each added complexity component
3. **Profile Performance:** Monitor computational requirements
4. **Adjust Based on Use Case:** Different applications need different fidelity levels

### Validation Strategies

Comprehensive validation approach:

1. **Unit Testing:** Test individual sensor models
2. **Integration Testing:** Test sensor fusion algorithms
3. **Field Testing:** Compare simulation results with real-world data
4. **Edge Case Testing:** Test extreme conditions and failure modes

### Documentation and Reproducibility

Ensure simulation results are reproducible:

1. **Parameter Documentation:** Document all simulation parameters
2. **Random Seed Management:** Use fixed seeds for reproducible results
3. **Version Control:** Track simulation model versions
4. **Benchmarking:** Establish performance baselines

## Future Trends

### AI-Enhanced Simulation

**Neural Radiance Fields (NeRF):**
- Photorealistic scene rendering
- Novel view synthesis
- Improved visual fidelity

**Generative Models:**
- Synthetic sensor data generation
- Domain adaptation techniques
- Style transfer for realism

### Physics-Based Rendering

**Advanced Light Transport:**
- Global illumination
- Subsurface scattering
- Realistic material properties

**Multi-Physics Simulation:**
- Integrated thermal, electromagnetic, and acoustic effects
- Realistic sensor-environment interactions
- Cross-modal sensor simulation

### Cloud-Based Simulation

**Distributed Computing:**
- Large-scale simulation environments
- Parallel sensor array simulation
- Real-time collaborative simulation

## Conclusion

Sensor simulation is a critical component of modern robotics development, enabling comprehensive testing and validation of perception and control algorithms before physical deployment. The key to successful sensor simulation lies in balancing model fidelity with computational performance while maintaining realistic sensor behaviors and noise characteristics. Modern simulation frameworks integrate multiple sensor types, incorporate sophisticated physical models, and provide tools for validation and optimization. As robotics applications become more complex and demanding, sensor simulation continues to evolve with advanced rendering techniques, AI-enhanced models, and cloud-based computing to meet the growing requirements of robotic system development and validation. The investment in realistic sensor simulation significantly reduces development time, costs, and risks while improving the safety and reliability of robotic systems in real-world applications.
---
sidebar_label: 'Simulation Realism'
title: 'Simulation Realism'
---

# Simulation Realism

## Introduction

Simulation realism refers to the degree to which a virtual environment accurately reproduces the physical, sensory, and behavioral characteristics of the real world. In robotics, achieving high-fidelity simulation realism is critical for developing, testing, and validating robotic systems before deployment in real-world environments. The ultimate goal is to minimize the "reality gap" – the difference between simulated and real-world performance – ensuring that algorithms and control strategies developed in simulation translate effectively to physical robots. This requires careful attention to physical properties, sensor behaviors, environmental conditions, and the complex interactions between these elements.

## Fundamentals of Simulation Realism

### Definition and Scope

Simulation realism encompasses multiple dimensions:

**Physical Realism:**
- Accurate modeling of physical laws (gravity, friction, collisions)
- Realistic material properties and responses
- Proper mass distribution and inertia tensors
- Correct force and torque calculations

**Sensory Realism:**
- Accurate reproduction of sensor noise and artifacts
- Realistic sensor limitations and characteristics
- Proper modeling of environmental effects on sensors
- Temporal coherence in sensor data

**Environmental Realism:**
- Faithful representation of real-world environments
- Accurate lighting, weather, and atmospheric conditions
- Proper modeling of environmental dynamics
- Realistic interaction between robot and environment

**Behavioral Realism:**
- Accurate modeling of system responses and delays
- Realistic actuator behaviors and limitations
- Proper modeling of system degradation and wear
- Accurate representation of real-world constraints

### The Reality Gap Problem

The reality gap represents the fundamental challenge in simulation-based robotics development:

**Sources of the Reality Gap:**
- Modeling simplifications and approximations
- Unknown or unmodeled physical phenomena
- Differences in environmental conditions
- Sensor model inaccuracies
- Actuator behavior mismatches
- Computational discretization effects

**Impact of the Reality Gap:**
- Algorithms that work in simulation fail in reality
- Control strategies that are stable in simulation become unstable in reality
- Performance metrics differ significantly between simulation and reality
- Safety margins calculated in simulation may not hold in reality

## Physical Realism

### Material Properties and Responses

Accurate material modeling is essential for realistic simulation:

**Density and Mass Distribution:**
```cpp
class MaterialPropertyModel {
private:
    struct Material {
        float density;                    // kg/m³
        float young_modulus;             // Pa (Young's modulus)
        float poisson_ratio;             // Dimensionless (-1 to 0.5)
        float yield_strength;            // Pa
        float ultimate_strength;         // Pa
        float thermal_expansion;         // 1/K
        float specific_heat;             // J/(kg·K)
        float thermal_conductivity;      // W/(m·K)
        float friction_coefficients[3];  // Static, kinetic, rolling
    };

    std::map<std::string, Material> materials_db_;

public:
    Material getMaterialProperties(const std::string& material_name) {
        auto it = materials_db_.find(material_name);
        if (it != materials_db_.end()) {
            return it->second;
        }
        return getDefaultMaterial();  // Return generic material if not found
    }

    float calculateEffectiveFriction(const std::string& mat1, const std::string& mat2) {
        // Calculate effective friction coefficient between two materials
        auto prop1 = getMaterialProperties(mat1);
        auto prop2 = getMaterialProperties(mat2);

        // Simple combination model (could be more sophisticated)
        return sqrt(prop1.friction_coefficients[0] * prop2.friction_coefficients[0]);
    }
};
```

**Surface Properties:**
```cpp
class SurfacePropertyModel {
public:
    struct SurfaceProperties {
        float roughness;          // Ra value (μm)
        float hardness;           // Vickers hardness (HV)
        float surface_energy;     // J/m²
        float reflectivity;       // 0.0 - 1.0
        float emissivity;         // 0.0 - 1.0
        float permeability;       // For magnetic materials
        float conductivity;       // S/m (electrical)
    };

    SurfaceProperties calculateContactProperties(
        const SurfaceProperties& surf1,
        const SurfaceProperties& surf2) {

        SurfaceProperties contact;
        // Combine properties using appropriate mixing rules
        contact.roughness = (surf1.roughness + surf2.roughness) / 2.0f;
        contact.hardness = std::min(surf1.hardness, surf2.hardness);
        contact.surface_energy = sqrt(surf1.surface_energy * surf2.surface_energy);
        contact.reflectivity = (surf1.reflectivity + surf2.reflectivity) / 2.0f;
        contact.emissivity = (surf1.emissivity + surf2.emissivity) / 2.0f;
        contact.permeability = sqrt(surf1.permeability * surf2.permeability);
        contact.conductivity = (surf1.conductivity + surf2.conductivity) / 2.0f;

        return contact;
    }
};
```

### Dynamic Behavior Modeling

Realistic modeling of dynamic responses:

**Viscoelastic Effects:**
```cpp
class ViscoelasticModel {
private:
    // Maxwell model parameters (spring-damper in series)
    float spring_constant_;    // N/m
    float damping_coeff_;      // N*s/m
    float relaxation_time_;    // s

    // Kelvin-Voigt model parameters (spring-damper in parallel)
    float kelvin_spring_;      // N/m
    float kelvin_damper_;      // N*s/m

public:
    float calculateForce(float displacement, float velocity, float time) {
        // Maxwell component (time-dependent)
        float maxwell_force = calculateMaxwellResponse(displacement, time);

        // Kelvin-Voigt component (rate-dependent)
        float kelvin_force = kelvin_spring_ * displacement + kelvin_damper_ * velocity;

        return maxwell_force + kelvin_force;
    }

private:
    float calculateMaxwellResponse(float displacement, float time) {
        // Stress relaxation in Maxwell model
        float initial_stress = spring_constant_ * displacement;
        float relaxation = exp(-time / relaxation_time_);
        return initial_stress * relaxation;
    }
};
```

**Contact Mechanics:**
```cpp
class ContactMechanicsModel {
public:
    struct ContactResult {
        Eigen::Vector3f contact_force;
        Eigen::Vector3f contact_torque;
        float penetration_depth;
        float contact_area;
        bool valid_contact;
    };

    ContactResult calculateContact(
        const CollisionObject& obj1,
        const CollisionObject& obj2,
        const ContactPoint& contact_point) {

        ContactResult result;
        result.valid_contact = false;

        // Calculate contact geometry
        float radius_curvature1 = calculateEffectiveCurvature(obj1, contact_point);
        float radius_curvature2 = calculateEffectiveCurvature(obj2, contact_point);

        // Hertzian contact model for elastic contact
        auto [contact_radius, penetration_depth] = calculateHertzianContact(
            obj1.material.young_modulus, obj2.material.young_modulus,
            obj1.material.poisson_ratio, obj2.material.poisson_ratio,
            radius_curvature1, radius_curvature2,
            obj1.material.yield_strength, obj2.material.yield_strength);

        if (penetration_depth > 0) {
            // Calculate normal contact force using Hertz model
            float normal_force = calculateHertzNormalForce(
                obj1.material.young_modulus, obj2.material.young_modulus,
                contact_radius, penetration_depth);

            // Calculate friction force
            float friction_coeff = calculateEffectiveFriction(obj1, obj2);
            Eigen::Vector3f relative_velocity = calculateRelativeVelocity(obj1, obj2, contact_point);

            // Apply friction model (Coulomb friction with Stribeck effect)
            Eigen::Vector3f friction_force = calculateFrictionForce(
                normal_force, friction_coeff, relative_velocity);

            result.contact_force = calculateNormalVector(obj1, obj2, contact_point) * normal_force + friction_force;
            result.penetration_depth = penetration_depth;
            result.contact_area = M_PI * contact_radius * contact_radius;
            result.valid_contact = true;
        }

        return result;
    }

private:
    std::pair<float, float> calculateHertzianContact(
        float E1, float E2, float nu1, float nu2,
        float R1, float R2, float Sy1, float Sy2) {

        // Combined Young's modulus
        float E_combined = 1.0f / ((1.0f - nu1*nu1) / E1 + (1.0f - nu2*nu2) / E2);

        // Combined radius
        float R_combined = 1.0f / (1.0f/R1 + 1.0f/R2);

        // Maximum contact pressure (for plasticity check)
        float p_max = sqrt(E_combined * pow(R_combined, -1) * pow(Sy1 + Sy2, 2) / 16.0f);

        // Contact radius and penetration depth
        float contact_radius = pow(3.0f * (Sy1 + Sy2) * R_combined / (4.0f * E_combined), 1.0f/3.0f);
        float penetration = pow(9.0f * pow(Sy1 + Sy2, 2) * R_combined / (16.0f * E_combined*E_combined), 1.0f/3.0f);

        return {contact_radius, penetration};
    }

    float calculateHertzNormalForce(float E_combined, float contact_radius, float penetration) {
        return (4.0f/3.0f) * E_combined * sqrt(contact_radius) * pow(penetration, 3.0f/2.0f);
    }

    float calculateEffectiveCurvature(const CollisionObject& obj, const ContactPoint& contact) {
        // Calculate effective radius of curvature at contact point
        // This would involve surface geometry analysis
        return obj.geometry.radius;  // Simplified for spherical approximation
    }

    float calculateEffectiveFriction(const CollisionObject& obj1, const CollisionObject& obj2) {
        // Combine friction coefficients using appropriate model
        return sqrt(obj1.material.friction_coefficient * obj2.material.friction_coefficient);
    }

    Eigen::Vector3f calculateRelativeVelocity(const CollisionObject& obj1, const CollisionObject& obj2, const ContactPoint& contact) {
        // Calculate relative velocity at contact point
        Eigen::Vector3f vel1 = obj1.velocity + obj1.angular_velocity.cross(contact.point - obj1.center_of_mass);
        Eigen::Vector3f vel2 = obj2.velocity + obj2.angular_velocity.cross(contact.point - obj2.center_of_mass);
        return vel1 - vel2;
    }

    Eigen::Vector3f calculateNormalVector(const CollisionObject& obj1, const CollisionObject& obj2, const ContactPoint& contact) {
        // Calculate contact normal (simplified)
        return (obj2.center_of_mass - obj1.center_of_mass).normalized();
    }

    Eigen::Vector3f calculateFrictionForce(float normal_force, float friction_coeff, const Eigen::Vector3f& relative_velocity) {
        float relative_speed = relative_velocity.norm();
        if (relative_speed < 1e-6) return Eigen::Vector3f::Zero();

        // Stribeck effect model
        float v_break = 0.01f;  // Breakaway velocity (m/s)
        float v_coulomb = 0.1f; // Coulomb friction velocity (m/s)

        float friction_multiplier = 1.0f;
        if (relative_speed < v_break) {
            // Stick regime
            friction_multiplier = 1.0f + (friction_coeff - 1.0f) * (relative_speed / v_break);
        } else if (relative_speed < v_coulomb) {
            // Stribeck regime (friction decreases with velocity)
            friction_multiplier = friction_coeff * (1.0f - 0.2f * (relative_speed - v_break) / (v_coulomb - v_break));
        } else {
            // Coulomb friction regime
            friction_multiplier = friction_coeff * 0.8f;  // Slightly reduced
        }

        Eigen::Vector3f tangential_velocity = relative_velocity - relative_velocity.dot(calculateNormalVector(obj1, obj2, contact)) * calculateNormalVector(obj1, obj2, contact);
        Eigen::Vector3f tangential_direction = tangential_velocity.normalized();

        float max_friction_force = friction_multiplier * normal_force;
        float actual_friction_force = std::min(max_friction_force, tangential_velocity.norm());

        return -tangential_direction * actual_friction_force;
    }
};
```

## Sensor Realism

### Noise Modeling

Realistic sensor noise modeling is crucial for simulation validity:

**Camera Sensor Noise:**
```cpp
class CameraNoiseModel {
public:
    struct NoiseParameters {
        float photon_shot_noise;     // σ_photon
        float readout_noise;         // σ_readout
        float dark_current_noise;    // σ_dark
        float quantization_noise;    // σ_quantization
        float fixed_pattern_noise;   // σ_pattern
        float temporal_dark_noise;   // σ_temporal_dark
    };

    cv::Mat addNoise(const cv::Mat& input_image, const NoiseParameters& params, double exposure_time) {
        cv::Mat noisy_image = input_image.clone();

        // Photon shot noise (signal-dependent)
        addPhotonShotNoise(noisy_image, params.photon_shot_noise, exposure_time);

        // Readout noise (additive Gaussian)
        addReadoutNoise(noisy_image, params.readout_noise);

        // Dark current noise (accumulates over time)
        addDarkCurrentNoise(noisy_image, params.dark_current_noise, exposure_time);

        // Quantization noise (due to finite bit depth)
        addQuantizationNoise(noisy_image, params.quantization_noise);

        // Fixed pattern noise (spatially correlated)
        addFixedPatternNoise(noisy_image, params.fixed_pattern_noise);

        // Temporal dark noise (time-varying dark current)
        addTemporalDarkNoise(noisy_image, params.temporal_dark_noise);

        return noisy_image;
    }

private:
    void addPhotonShotNoise(cv::Mat& image, float base_noise, double exposure_time) {
        for (int y = 0; y < image.rows; ++y) {
            for (int x = 0; x < image.cols; ++x) {
                cv::Vec3b& pixel = image.at<cv::Vec3b>(y, x);

                // Signal-dependent noise: σ = sqrt(signal)
                float noise_r = generatePoissonNoise(pixel[2] * base_noise * exposure_time);
                float noise_g = generatePoissonNoise(pixel[1] * base_noise * exposure_time);
                float noise_b = generatePoissonNoise(pixel[0] * base_noise * exposure_time);

                pixel[2] = saturate_cast<uchar>(pixel[2] + noise_r);
                pixel[1] = saturate_cast<uchar>(pixel[1] + noise_g);
                pixel[0] = saturate_cast<uchar>(pixel[0] + noise_b);
            }
        }
    }

    void addReadoutNoise(cv::Mat& image, float noise_std) {
        cv::Mat noise(image.size(), image.type());
        cv::randn(noise, cv::Scalar::all(0), cv::Scalar::all(noise_std));
        cv::add(image, noise, image);
    }

    void addDarkCurrentNoise(cv::Mat& image, float dark_current_rate, double exposure_time) {
        // Dark current accumulates over exposure time
        cv::Mat dark_noise(image.size(), image.type());
        cv::randn(dark_noise, cv::Scalar::all(0), cv::Scalar::all(dark_current_rate * exposure_time));
        cv::add(image, dark_noise, image);
    }

    void addQuantizationNoise(cv::Mat& image, float q) {
        // Quantization noise = q^2/12 (where q is quantum size)
        cv::Mat quant_noise(image.size(), image.type());
        cv::randn(quant_noise, cv::Scalar::all(0), cv::Scalar::all(q / sqrt(12.0)));
        cv::add(image, quant_noise, image);
    }

    void addFixedPatternNoise(cv::Mat& image, float noise_std) {
        // Spatially correlated noise pattern (remains constant across frames)
        static cv::Mat pattern_noise;
        if (pattern_noise.empty()) {
            pattern_noise = cv::Mat::zeros(image.size(), CV_32F);
            cv::randn(pattern_noise, cv::Scalar::all(0), cv::Scalar::all(noise_std));
        }

        cv::Mat temp;
        image.convertTo(temp, CV_32F);
        cv::add(temp, pattern_noise, temp);
        temp.convertTo(image, image.type());
    }

    void addTemporalDarkNoise(cv::Mat& image, float noise_std) {
        // Time-varying dark current pattern
        cv::Mat temporal_noise(image.size(), image.type());
        cv::randn(temporal_noise, cv::Scalar::all(0), cv::Scalar::all(noise_std));
        cv::add(image, temporal_noise, image);
    }

    float generatePoissonNoise(float lambda) {
        // Generate noise based on Poisson distribution (simplified)
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::poisson_distribution<> pois_dist;

        // For large lambda, approximate with normal distribution
        if (lambda > 30) {
            std::normal_distribution<float> norm_dist(lambda, sqrt(lambda));
            return norm_dist(gen);
        } else {
            pois_dist.param(std::poisson_distribution<>::param_type(lambda));
            return pois_dist(gen);
        }
    }
};
```

**LiDAR Sensor Realism:**
```cpp
class LiDARRealismModel {
public:
    struct LiDARParameters {
        float range_accuracy;        // Range measurement accuracy (m)
        float angular_accuracy;      // Angular measurement accuracy (rad)
        float intensity_accuracy;    // Intensity measurement accuracy
        float multi_path_prob;       // Probability of multi-path returns
        float drop_out_prob;         // Probability of missed returns
        float ghost_return_prob;     // Probability of phantom returns
        float ambient_light_effect;  // Effect of ambient light on intensity
        float temperature_drift;     // Range drift due to temperature (m/K)
    };

    struct LiDARData {
        std::vector<float> ranges;
        std::vector<float> intensities;
        std::vector<Eigen::Vector3f> points;
        double timestamp;
    };

    LiDARData addRealismEffects(const LiDARData& raw_data,
                               const LiDARParameters& params,
                               const EnvironmentConditions& env,
                               double temperature) {
        LiDARData realistic_data = raw_data;

        // Apply range accuracy errors
        applyRangeErrors(realistic_data.ranges, params.range_accuracy);

        // Apply angular accuracy errors
        applyAngularErrors(realistic_data.points, params.angular_accuracy);

        // Apply intensity accuracy errors
        applyIntensityErrors(realistic_data.intensities, params.intensity_accuracy);

        // Apply multi-path effects
        applyMultiPathEffects(realistic_data, params.multi_path_prob);

        // Apply drop-out effects
        applyDropOutEffects(realistic_data, params.drop_out_prob);

        // Apply ghost return effects
        applyGhostReturns(realistic_data, params.ghost_return_prob);

        // Apply ambient light effects
        applyAmbientLightEffects(realistic_data.intensities, env.ambient_light, params.ambient_light_effect);

        // Apply temperature drift
        applyTemperatureDrift(realistic_data.ranges, temperature, params.temperature_drift);

        // Apply atmospheric effects (weather, humidity)
        applyAtmosphericEffects(realistic_data, env);

        return realistic_data;
    }

private:
    void applyRangeErrors(std::vector<float>& ranges, float accuracy_std) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::normal_distribution<float> dist(0.0f, accuracy_std);

        for (float& range : ranges) {
            if (range > 0) {  // Only apply to valid measurements
                range += dist(gen);
                range = std::max(0.0f, range);  // Ensure non-negative
            }
        }
    }

    void applyAngularErrors(std::vector<Eigen::Vector3f>& points, float angular_std) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::normal_distribution<float> dist(0.0f, angular_std);

        for (auto& point : points) {
            if (point.norm() > 0) {
                // Apply angular perturbations
                float delta_azimuth = dist(gen);
                float delta_elevation = dist(gen);

                // Convert to spherical coordinates
                float r = point.norm();
                float azimuth = atan2(point.y(), point.x());
                float elevation = asin(point.z() / r);

                // Apply angular errors
                azimuth += delta_azimuth;
                elevation += delta_elevation;

                // Convert back to Cartesian
                point.x() = r * cos(elevation) * cos(azimuth);
                point.y() = r * cos(elevation) * sin(azimuth);
                point.z() = r * sin(elevation);
            }
        }
    }

    void applyIntensityErrors(std::vector<float>& intensities, float accuracy_std) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::normal_distribution<float> dist(0.0f, accuracy_std);

        for (float& intensity : intensities) {
            if (intensity > 0) {
                intensity += dist(gen);
                intensity = std::max(0.0f, intensity);  // Ensure non-negative
            }
        }
    }

    void applyMultiPathEffects(LiDARData& data, float multi_path_prob) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> uni_dist(0.0f, 1.0f);

        for (size_t i = 0; i < data.ranges.size(); ++i) {
            if (uni_dist(gen) < multi_path_prob) {
                // Simulate multi-path return: slightly longer range with reduced intensity
                float original_range = data.ranges[i];
                float original_intensity = data.intensities[i];

                // Multi-path typically results in longer path
                data.ranges[i] = original_range + generateMultiPathDelta();

                // Reduced intensity due to additional reflection
                data.intensities[i] = original_intensity * 0.7f;  // 30% reduction

                // Add secondary return if detector can resolve it
                if (uni_dist(gen) < 0.3f) {  // 30% chance of detecting secondary return
                    // Insert secondary return at slightly different angle
                    // This would require expanding the data structure
                }
            }
        }
    }

    void applyDropOutEffects(LiDARData& data, float drop_out_prob) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> uni_dist(0.0f, 1.0f);

        for (size_t i = 0; i < data.ranges.size(); ++i) {
            if (uni_dist(gen) < drop_out_prob) {
                // Mark as invalid return (drop-out)
                data.ranges[i] = 0;  // Invalid range
                data.intensities[i] = 0;  // No return
            }
        }
    }

    void applyGhostReturns(LiDARData& data, float ghost_prob) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> uni_dist(0.0f, 1.0f);

        // Ghost returns are spurious returns that don't correspond to real objects
        for (size_t i = 0; i < data.ranges.size(); ++i) {
            if (uni_dist(gen) < ghost_prob) {
                // Add a ghost return at a random range
                data.ranges[i] = generateRandomRange();  // Random plausible range
                data.intensities[i] = generateRandomIntensity();  // Low intensity ghost
            }
        }
    }

    void applyAmbientLightEffects(std::vector<float>& intensities,
                                 float ambient_light,
                                 float effect_coeff) {
        // Ambient light reduces dynamic range and contrast
        float ambient_effect = effect_coeff * ambient_light;

        for (float& intensity : intensities) {
            if (intensity > 0) {
                // Reduce intensity contrast in bright conditions
                intensity = intensity * (1.0f - ambient_effect) + ambient_effect * 0.1f;
            }
        }
    }

    void applyTemperatureDrift(std::vector<float>& ranges, double temperature, float drift_coeff) {
        // Range drift due to temperature effects on timing circuits
        static double reference_temperature = 20.0;  // Reference temperature in Celsius

        float temperature_delta = static_cast<float>(temperature - reference_temperature);
        float drift = drift_coeff * temperature_delta;

        for (float& range : ranges) {
            if (range > 0) {
                range += drift;
            }
        }
    }

    void applyAtmosphericEffects(LiDARData& data, const EnvironmentConditions& env) {
        // Apply effects of weather conditions on LiDAR performance
        float extinction_coeff = calculateExtinctionCoefficient(env);

        for (size_t i = 0; i < data.ranges.size(); ++i) {
            if (data.ranges[i] > 0) {
                // Range attenuation due to atmospheric extinction
                float attenuation = exp(-extinction_coeff * data.ranges[i]);

                // Reduce intensity based on atmospheric attenuation
                data.intensities[i] *= attenuation;

                // Increase range measurement error in poor visibility
                float visibility_factor = calculateVisibilityFactor(env.visibility);
                float range_error_increase = (1.0f - visibility_factor) * 0.1f;  // Up to 10% additional error

                static std::random_device rd;
                static std::mt19937 gen(rd());
                std::normal_distribution<float> dist(0.0f, range_error_increase);

                data.ranges[i] += dist(gen);
            }
        }
    }

    float calculateExtinctionCoefficient(const EnvironmentConditions& env) {
        // Simplified extinction coefficient calculation
        // In reality, this would be more complex and wavelength-dependent
        float base_extinction = 0.0;  // Clear air extinction

        // Add effects of fog, rain, dust, etc.
        base_extinction += env.fog_density * 0.1f;  // Fog contribution
        base_extinction += env.rain_rate * 0.05f;   // Rain contribution
        base_extinction += env.dust_density * 0.02f; // Dust contribution

        return base_extinction;
    }

    float calculateVisibilityFactor(float visibility) {
        // Convert visibility to a factor between 0 and 1
        // Lower visibility = lower factor
        return std::min(1.0f, visibility / 100.0f);  // Assuming 100m as "perfect" visibility
    }

    float generateMultiPathDelta() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::normal_distribution<float> dist(0.01f, 0.005f);  // Small additional range
        return std::abs(dist(gen));  // Always positive
    }

    float generateRandomRange() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(0.1f, 30.0f);  // Typical LiDAR range
        return dist(gen);
    }

    float generateRandomIntensity() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(0.0f, 0.3f);  // Low intensity for ghosts
        return dist(gen);
    }
};

struct EnvironmentConditions {
    float ambient_light;     // Lux
    float visibility;        // Meters
    float fog_density;       // Dimensionless (0-1)
    float rain_rate;         // mm/hour
    float dust_density;      // Particles/m³
    float temperature;       // Celsius
    float humidity;          // Percent (0-100)
    float atmospheric_pressure; // kPa
};
```

## Environmental Realism

### Lighting and Visual Effects

Realistic lighting simulation affects computer vision algorithms:

```cpp
class LightingRealismModel {
public:
    struct LightSource {
        enum Type { DIRECTIONAL, POINT, SPOT, AREA };
        Type type;
        Eigen::Vector3f position;
        Eigen::Vector3f direction;
        Eigen::Vector3f color;  // RGB in [0,1]
        float intensity;        // cd or W
        float range;            // For point/spot lights
        float spot_angle;       // For spot lights
    };

    struct AtmosphericConditions {
        float turbidity;        // Atmospheric clarity (2.0-7.0)
        float ozone_thickness;  // Dobson units
        float water_vapor;      // cm precipitable water
        float aerosol_optical_depth;  // At 550nm
        float albedo;           // Ground albedo (0-1)
    };

    cv::Mat simulateLighting(const Scene& scene,
                            const CameraParameters& camera,
                            const std::vector<LightSource>& lights,
                            const AtmosphericConditions& atm) {

        cv::Mat rendered_image = renderScene(scene, camera, lights);

        // Apply atmospheric effects
        rendered_image = applyAtmosphericScattering(rendered_image, atm, camera);

        // Apply lens effects
        rendered_image = applyLensEffects(rendered_image, camera);

        // Apply sensor-specific effects
        rendered_image = applySensorEffects(rendered_image, camera.sensor);

        return rendered_image;
    }

private:
    cv::Mat renderScene(const Scene& scene,
                       const CameraParameters& camera,
                       const std::vector<LightSource>& lights) {
        // Ray tracing or rasterization-based rendering
        cv::Mat image(camera.height, camera.width, CV_8UC3, cv::Scalar(0,0,0));

        for (int y = 0; y < camera.height; ++y) {
            for (int x = 0; x < camera.width; ++x) {
                // Calculate ray through pixel
                Ray ray = calculateCameraRay(x, y, camera);

                // Ray-scene intersection
                IntersectionResult hit = scene.rayIntersect(ray.origin, ray.direction);

                if (hit.valid) {
                    // Calculate illumination at intersection point
                    cv::Vec3f pixel_color = calculateIllumination(hit, scene, lights);
                    image.at<cv::Vec3b>(y, x) = convertToByteColor(pixel_color);
                } else {
                    // Background/sky color
                    image.at<cv::Vec3b>(y, x) = calculateSkyColor(ray.direction, lights);
                }
            }
        }

        return image;
    }

    cv::Vec3f calculateIllumination(const IntersectionResult& hit,
                                   const Scene& scene,
                                   const std::vector<LightSource>& lights) {
        cv::Vec3f total_illumination(0, 0, 0);

        // Ambient lighting
        total_illumination += hit.material.ambient_color * scene.ambient_light_intensity;

        for (const auto& light : lights) {
            // Calculate light contribution
            float visibility = calculateShadow(hit.point, light);

            if (visibility > 0) {
                // Diffuse lighting (Lambert's cosine law)
                float diffuse_factor = std::max(0.0f, hit.normal.dot(light.direction));
                cv::Vec3f diffuse = hit.material.diffuse_color.mul(cv::Vec3f(light.color)) *
                                   light.intensity * diffuse_factor * visibility;

                // Specular lighting (Blinn-Phong model)
                Eigen::Vector3f view_dir = (camera.position - hit.point).normalized();
                Eigen::Vector3f half_vector = (light.direction + view_dir.cast<float>()).normalized();
                float specular_factor = pow(std::max(0.0f, hit.normal.dot(half_vector.cast<float>())),
                                           hit.material.shininess);
                cv::Vec3f specular = hit.material.specular_color.mul(cv::Vec3f(light.color)) *
                                    light.intensity * specular_factor * visibility;

                total_illumination += diffuse + specular;
            }
        }

        // Clamp to reasonable range
        total_illumination[0] = std::min(1.0f, total_illumination[0]);
        total_illumination[1] = std::min(1.0f, total_illumination[1]);
        total_illumination[2] = std::min(1.0f, total_illumination[2]);

        return total_illumination;
    }

    cv::Mat applyAtmosphericScattering(const cv::Mat& image,
                                      const AtmosphericConditions& atm,
                                      const CameraParameters& camera) {
        // Apply atmospheric scattering effects (haze, fog, etc.)
        cv::Mat scattered_image = image.clone();

        for (int y = 0; y < image.rows; ++y) {
            for (int x = 0; x < image.cols; ++x) {
                cv::Vec3b& pixel = scattered_image.at<cv::Vec3b>(y, x);

                // Calculate distance-based atmospheric effects
                float distance = calculatePixelDistance(x, y, camera);

                // Atmospheric extinction coefficient
                float beta = calculateExtinctionCoeff(atm);

                // Transmission (Beer-Lambert law)
                float transmission = exp(-beta * distance);

                // Apply atmospheric scattering
                cv::Vec3f original_pixel = cv::Vec3f(pixel[2]/255.0f, pixel[1]/255.0f, pixel[0]/255.0f);
                cv::Vec3f atmospheric_light = calculateAtmosphericLight(atm);

                cv::Vec3f scattered_color = original_pixel * transmission +
                                          atmospheric_light * (1.0f - transmission);

                pixel[2] = static_cast<uchar>(scattered_color[0] * 255.0f);
                pixel[1] = static_cast<uchar>(scattered_color[1] * 255.0f);
                pixel[0] = static_cast<uchar>(scattered_color[2] * 255.0f);
            }
        }

        return scattered_image;
    }

    float calculateExtinctionCoeff(const AtmosphericConditions& atm) {
        // Calculate extinction coefficient based on atmospheric composition
        // β = β_rayleigh + β_ozone + β_aerosol + β_water
        float beta_rayleigh = 0.0115f;  // Simplified Rayleigh scattering
        float beta_ozone = atm.ozone_thickness * 0.00035f;  // Ozone absorption
        float beta_aerosol = atm.aerosol_optical_depth * 3.91f / 20.0f;  // Visibility-based
        float beta_water = atm.water_vapor * 0.01f;  // Water vapor absorption

        return beta_rayleigh + beta_ozone + beta_aerosol + beta_water;
    }

    cv::Vec3f calculateAtmosphericLight(const AtmosphericConditions& atm) {
        // Calculate color of atmospheric light (skylight)
        // This is a simplified model - real models are more complex
        float turbidity_factor = std::min(2.0f, atm.turbidity / 3.0f);

        cv::Vec3f skylight(0.5f, 0.6f, 0.8f);  // Blue-ish sky
        skylight *= (1.0f - turbidity_factor * 0.3f);  // More white with higher turbidity

        return skylight;
    }

    cv::Mat applyLensEffects(const cv::Mat& image, const CameraParameters& camera) {
        cv::Mat lens_corrected = image.clone();

        // Apply lens distortion
        cv::Mat distorted;
        cv::undistort(image, distorted, camera.intrinsic_matrix, camera.distortion_coeffs);

        // Apply vignetting (corner darkening)
        applyVignetting(distorted, camera.vignetting_coeff);

        // Apply chromatic aberration
        applyChromaticAberration(distorted, camera.chromatic_aberration);

        return distorted;
    }

    void applyVignetting(cv::Mat& image, float coeff) {
        int width = image.cols;
        int height = image.rows;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Calculate distance from center
                float center_x = width / 2.0f;
                float center_y = height / 2.0f;
                float dx = x - center_x;
                float dy = y - center_y;
                float distance = sqrt(dx*dx + dy*dy);
                float max_distance = sqrt(center_x*center_x + center_y*center_y);
                float normalized_distance = distance / max_distance;

                // Vignetting factor
                float vignette_factor = 1.0f - coeff * normalized_distance * normalized_distance;
                vignette_factor = std::max(0.0f, std::min(1.0f, vignette_factor));

                cv::Vec3b& pixel = image.at<cv::Vec3b>(y, x);
                pixel[0] = static_cast<uchar>(pixel[0] * vignette_factor);
                pixel[1] = static_cast<uchar>(pixel[1] * vignette_factor);
                pixel[2] = static_cast<uchar>(pixel[2] * vignette_factor);
            }
        }
    }

    void applyChromaticAberration(cv::Mat& image, float coeff) {
        // Simplified chromatic aberration model
        // Different wavelengths focus at slightly different positions
        cv::Mat channels[3];
        cv::split(image, channels);

        // Apply different radial distortions to each channel
        cv::Mat r_channel, g_channel, b_channel;

        // Red channel (longest wavelength) - more aberration
        applyRadialShift(channels[2], r_channel, coeff * 1.0f);

        // Green channel (middle wavelength) - medium aberration
        applyRadialShift(channels[1], g_channel, coeff * 0.7f);

        // Blue channel (shortest wavelength) - less aberration
        applyRadialShift(channels[0], b_channel, coeff * 0.5f);

        cv::merge({b_channel, g_channel, r_channel}, image);
    }

    void applyRadialShift(const cv::Mat& input, cv::Mat& output, float shift_coeff) {
        output = input.clone();
        int width = input.cols;
        int height = input.rows;

        float center_x = width / 2.0f;
        float center_y = height / 2.0f;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float dx = x - center_x;
                float dy = y - center_y;
                float distance = sqrt(dx*dx + dy*dy);
                float max_distance = sqrt(center_x*center_x + center_y*center_y);

                // Calculate shift amount based on distance from center
                float shift_amount = shift_coeff * (distance / max_distance) * 5.0f;  // Max 5 pixel shift

                float shifted_x = x - (dx / distance) * shift_amount;
                float shifted_y = y - (dy / distance) * shift_amount;

                // Sample from shifted position using bilinear interpolation
                if (shifted_x >= 0 && shifted_x < width-1 && shifted_y >= 0 && shifted_y < height-1) {
                    output.at<uchar>(y, x) = bilinearSample(input, shifted_x, shifted_y);
                }
            }
        }
    }

    uchar bilinearSample(const cv::Mat& img, float x, float y) {
        int x1 = static_cast<int>(floor(x));
        int y1 = static_cast<int>(floor(y));
        int x2 = x1 + 1;
        int y2 = y1 + 1;

        float dx = x - x1;
        float dy = y - y1;

        if (x2 >= img.cols) x2 = img.cols - 1;
        if (y2 >= img.rows) y2 = img.rows - 1;

        uchar val1 = img.at<uchar>(y1, x1);
        uchar val2 = img.at<uchar>(y1, x2);
        uchar val3 = img.at<uchar>(y2, x1);
        uchar val4 = img.at<uchar>(y2, x2);

        float result = val1 * (1-dx) * (1-dy) +
                      val2 * dx * (1-dy) +
                      val3 * (1-dx) * dy +
                      val4 * dx * dy;

        return static_cast<uchar>(result);
    }

    cv::Mat applySensorEffects(const cv::Mat& image, const SensorParameters& sensor) {
        cv::Mat sensor_image = image.clone();

        // Apply sensor-specific noise model
        CameraNoiseModel noise_model;
        auto noise_params = convertToNoiseParams(sensor);
        sensor_image = noise_model.addNoise(sensor_image, noise_params, sensor.exposure_time);

        // Apply gamma correction
        applyGammaCorrection(sensor_image, sensor.gamma);

        // Apply color filter array effects (Bayer pattern)
        if (sensor.bayer_pattern != SensorParameters::NONE) {
            sensor_image = applyBayerDemosaicing(sensor_image, sensor.bayer_pattern);
        }

        return sensor_image;
    }

    cv::Mat applyBayerDemosaicing(const cv::Mat& bayer_image,
                                 SensorParameters::BayerPattern pattern) {
        // Apply Bayer demosaicing to convert from single-channel Bayer to RGB
        cv::Mat rgb_image;

        int code = -1;
        switch (pattern) {
            case SensorParameters::RGGB: code = cv::COLOR_BayerBG2BGR; break;
            case SensorParameters::GRBG: code = cv::COLOR_BayerGB2BGR; break;
            case SensorParameters::BGGR: code = cv::COLOR_BayerRG2BGR; break;
            case SensorParameters::GBRG: code = cv::COLOR_BayerGR2BGR; break;
        }

        if (code != -1) {
            cv::cvtColor(bayer_image, rgb_image, code);
        } else {
            rgb_image = bayer_image;  // No conversion if pattern not recognized
        }

        return rgb_image;
    }

    void applyGammaCorrection(cv::Mat& image, float gamma) {
        cv::Mat lut(1, 256, CV_8UC1);
        for (int i = 0; i < 256; ++i) {
            lut.at<uchar>(i) = cv::saturate_cast<uchar>(pow(i / 255.0, 1.0/gamma) * 255.0);
        }

        cv::LUT(image, lut, image);
    }

    float calculatePixelDistance(int x, int y, const CameraParameters& camera) {
        // Calculate distance from camera to scene point at pixel (x,y)
        // This is a simplified calculation - in reality, this would require
        // depth information from the scene
        return 10.0f;  // Default distance for background pixels
    }

    cv::Vec3b calculateSkyColor(const Eigen::Vector3f& ray_dir, const std::vector<LightSource>& lights) {
        // Calculate sky color based on ray direction and light sources
        // Simplified model - real sky models are more complex

        // Find dominant directional light (usually sun)
        Eigen::Vector3f sun_dir(0, 0, -1);  // Default sun direction
        for (const auto& light : lights) {
            if (light.type == LightSource::DIRECTIONAL) {
                sun_dir = light.direction.normalized();
                break;
            }
        }

        // Sky color varies with angle from sun
        float cos_theta = ray_dir.dot(-sun_dir);
        float angle_factor = (cos_theta + 1.0f) / 2.0f;

        // Blue sky with sun glare
        cv::Vec3b sky_color;
        sky_color[0] = static_cast<uchar>(150 + 50 * angle_factor);  // Blue increases toward sun
        sky_color[1] = static_cast<uchar>(180 + 40 * angle_factor);  // Green increases toward sun
        sky_color[2] = static_cast<uchar>(200 + 30 * angle_factor);  // Red increases toward sun

        return sky_color;
    }

    float calculateShadow(const Eigen::Vector3f& point, const LightSource& light) {
        // Simplified shadow calculation
        // In reality, this would involve complex shadow mapping or ray tracing
        return 1.0f;  // No shadows for simplicity
    }

    cv::Vec3f convertToByteColor(const cv::Vec3f& float_color) {
        return cv::Vec3f(
            std::min(255.0f, float_color[0] * 255.0f),
            std::min(255.0f, float_color[1] * 255.0f),
            std::min(255.0f, float_color[2] * 255.0f)
        );
    }

    struct Ray {
        Eigen::Vector3f origin;
        Eigen::Vector3f direction;
    };

    Ray calculateCameraRay(int x, int y, const CameraParameters& camera) {
        // Convert pixel coordinates to world space ray
        float u = (x - camera.cx) / camera.fx;
        float v = (y - camera.cy) / camera.fy;

        Eigen::Vector3f ray_dir(u, v, 1.0f);
        ray_dir.normalize();

        // Transform from camera space to world space
        Eigen::Vector3f world_ray = camera.extrinsics.rotation * ray_dir;
        Eigen::Vector3f world_origin = camera.extrinsics.translation;

        return {world_origin, world_ray};
    }

    struct CameraParameters {
        float fx, fy;  // Focal lengths
        float cx, cy;  // Principal point
        cv::Mat intrinsic_matrix;
        cv::Mat distortion_coeffs;
        float vignetting_coeff;
        float chromatic_aberration;
        float gamma;
        int width, height;
        struct Extrinsics {
            Eigen::Matrix3f rotation;
            Eigen::Vector3f translation;
        } extrinsics;
        struct SensorParams {
            float exposure_time;
            float sensitivity_iso;
            float dynamic_range_db;
        } sensor;
    };

    struct SensorParameters {
        enum BayerPattern { NONE, RGGB, GRBG, BGGR, GBRG };
        float exposure_time;
        float gamma;
        float readout_noise;
        float dark_current;
        float quantization_noise;
        BayerPattern bayer_pattern;
    };

    SensorParameters::BayerPattern convertBayerPattern(int pattern_code) {
        // Convert from sensor code to enum
        return SensorParameters::RGGB;  // Default
    }

    CameraNoiseModel::NoiseParameters convertToNoiseParams(const SensorParameters& sensor) {
        CameraNoiseModel::NoiseParameters params;
        params.readout_noise = sensor.readout_noise;
        params.dark_current_noise = sensor.dark_current;
        params.quantization_noise = sensor.quantization_noise;
        params.photon_shot_noise = 0.1f;  // Default
        params.fixed_pattern_noise = 0.05f;  // Default
        params.temporal_dark_noise = 0.01f;  // Default

        return params;
    }
};
```

### Weather and Environmental Effects

Modeling environmental conditions that affect sensor performance:

```cpp
class WeatherEffectModel {
public:
    struct WeatherConditions {
        enum PrecipitationType { NONE, RAIN, SNOW, HAIL };
        PrecipitationType precipitation;
        float precipitation_rate;    // mm/hour
        float visibility;           // meters
        float fog_density;          // 0.0-1.0
        float wind_speed;           // m/s
        float temperature;          // Celsius
        float humidity;             // 0-100%
        float atmospheric_pressure; // kPa
        float uv_index;             // 0-11+
    };

    cv::Mat applyWeatherEffects(const cv::Mat& input_image,
                               const WeatherConditions& weather,
                               const SensorParameters& sensor) {
        cv::Mat weather_effected = input_image.clone();

        // Apply visibility effects
        weather_effected = applyVisibilityEffects(weather_effected, weather.visibility);

        // Apply precipitation effects
        weather_effected = applyPrecipitationEffects(weather_effected, weather, sensor);

        // Apply fog effects
        weather_effected = applyFogEffects(weather_effected, weather.fog_density);

        // Apply temperature effects on sensor
        weather_effected = applyTemperatureEffects(weather_effected, weather.temperature, sensor);

        // Apply humidity effects
        weather_effected = applyHumidityEffects(weather_effected, weather.humidity);

        return weather_effected;
    }

private:
    cv::Mat applyVisibilityEffects(const cv::Mat& image, float visibility) {
        cv::Mat visibility_effected = image.clone();

        // Calculate atmospheric extinction based on visibility
        float extinction_coeff = 3.91f / std::max(visibility, 1.0f);  // Koschmieder's law

        // Apply distance-based atmospheric extinction
        for (int y = 0; y < image.rows; ++y) {
            for (int x = 0; x < image.cols; ++x) {
                cv::Vec3b& pixel = visibility_effected.at<cv::Vec3b>(y, x);

                // Assume distance increases from top to bottom of image
                float distance_factor = static_cast<float>(y) / image.rows;
                float distance = distance_factor * 50.0f;  // Max distance of 50m

                float transmission = exp(-extinction_coeff * distance);
                float scatter_factor = 1.0f - transmission;

                // Apply extinction and add atmospheric (hazy) component
                cv::Vec3b atmospheric_component(150, 160, 170);  // Grayish haze

                pixel[0] = static_cast<uchar>(pixel[0] * transmission + atmospheric_component[0] * scatter_factor);
                pixel[1] = static_cast<uchar>(pixel[1] * transmission + atmospheric_component[1] * scatter_factor);
                pixel[2] = static_cast<uchar>(pixel[2] * transmission + atmospheric_component[2] * scatter_factor);
            }
        }

        return visibility_effected;
    }

    cv::Mat applyPrecipitationEffects(const cv::Mat& image,
                                     const WeatherConditions& weather,
                                     const SensorParameters& sensor) {
        cv::Mat precip_effected = image.clone();

        switch (weather.precipitation) {
            case WeatherConditions::RAIN:
                precip_effected = addRainDrops(precip_effected, weather.precipitation_rate, sensor);
                precip_effected = addRainStreaks(precip_effected, weather.wind_speed);
                break;
            case WeatherConditions::SNOW:
                precip_effected = addSnowFlakes(precip_effected, weather.precipitation_rate, weather.wind_speed);
                break;
            case WeatherConditions::HAIL:
                precip_effected = addHailImpact(precip_effected, weather.precipitation_rate);
                break;
            case WeatherConditions::NONE:
            default:
                break;
        }

        return precip_effected;
    }

    cv::Mat addRainDrops(const cv::Mat& image, float rainfall_rate, const SensorParameters& sensor) {
        cv::Mat rain_effected = image.clone();

        // Calculate number of raindrops based on rainfall rate
        int num_drops = static_cast<int>(rainfall_rate * 0.1);  // Simplified scaling

        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<int> x_dist(0, image.cols - 1);
        std::uniform_int_distribution<int> y_dist(0, image.rows - 1);
        std::uniform_real_distribution<float> size_dist(1.0f, 5.0f);

        for (int i = 0; i < num_drops; ++i) {
            int x = x_dist(gen);
            int y = y_dist(gen);
            float radius = size_dist(gen);

            // Raindrop creates refraction/distortion effect
            applyRainDropEffect(rain_effected, x, y, radius);
        }

        return rain_effected;
    }

    void applyRainDropEffect(cv::Mat& image, int center_x, int center_y, float radius) {
        int x_min = std::max(0, static_cast<int>(center_x - radius));
        int x_max = std::min(image.cols - 1, static_cast<int>(center_x + radius));
        int y_min = std::max(0, static_cast<int>(center_y - radius));
        int y_max = std::min(image.rows - 1, static_cast<int>(center_y + radius));

        for (int y = y_min; y <= y_max; ++y) {
            for (int x = x_min; x <= x_max; ++x) {
                float dx = x - center_x;
                float dy = y - center_y;
                float distance = sqrt(dx*dx + dy*dy);

                if (distance <= radius) {
                    // Apply refraction effect (simplified)
                    float refraction_factor = 1.0f + 0.2f * (distance / radius);

                    // Displace pixel sampling location
                    int displaced_x = static_cast<int>(center_x + dx * refraction_factor);
                    int displaced_y = static_cast<int>(center_y + dy * refraction_factor);

                    displaced_x = std::max(0, std::min(image.cols - 1, displaced_x));
                    displaced_y = std::max(0, std::min(image.rows - 1, displaced_y));

                    image.at<cv::Vec3b>(y, x) = image.at<cv::Vec3b>(displaced_y, displaced_x);
                }
            }
        }
    }

    cv::Mat addRainStreaks(const cv::Mat& image, float wind_speed) {
        cv::Mat streaked_image = image.clone();

        // Rain streaks due to motion blur
        float streak_length = std::min(20.0f, wind_speed * 2.0f);  // Max 20 pixels

        if (streak_length > 1.0f) {
            // Apply motion blur in direction opposite to wind
            cv::Mat kernel = cv::Mat::zeros(1, static_cast<int>(streak_length), CV_32F);
            for (int i = 0; i < kernel.cols; ++i) {
                kernel.at<float>(0, i) = 1.0f / kernel.cols;
            }

            cv::filter2D(streaked_image, streaked_image, -1, kernel);
        }

        return streaked_image;
    }

    cv::Mat addSnowFlakes(const cv::Mat& image, float snow_rate, float wind_speed) {
        cv::Mat snowed_image = image.clone();

        // Calculate number of snowflakes based on snow rate
        int num_flakes = static_cast<int>(snow_rate * 0.05);  // Simplified scaling

        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<int> x_dist(0, image.cols - 1);
        std::uniform_int_distribution<int> y_dist(0, image.rows - 1);

        for (int i = 0; i < num_flakes; ++i) {
            int x = x_dist(gen);
            int y = y_dist(gen);

            // Snowflake appears as bright spot with soft edges
            addSnowFlake(snowed_image, x, y);
        }

        return snowed_image;
    }

    void addSnowFlake(cv::Mat& image, int x, int y) {
        // Draw a soft, bright snowflake
        cv::circle(image, cv::Point(x, y), 2, cv::Scalar(255, 255, 255), -1);

        // Add soft glow effect
        for (int r = 3; r <= 5; ++r) {
            float intensity = 200.0f * (5 - r) / 5.0f;
            cv::circle(image, cv::Point(x, y), r, cv::Scalar(intensity, intensity, intensity), 1);
        }
    }

    cv::Mat applyFogEffects(const cv::Mat& image, float fog_density) {
        cv::Mat fogged_image = image.clone();

        if (fog_density > 0) {
            // Apply fog as distance-based atmospheric scattering
            cv::Vec3b fog_color(180, 190, 200);  // Light gray fog

            for (int y = 0; y < image.rows; ++y) {
                for (int x = 0; x < image.cols; ++x) {
                    cv::Vec3b& pixel = fogged_image.at<cv::Vec3b>(y, x);

                    // Distance increases from foreground to background
                    float distance_factor = static_cast<float>(y) / image.rows;
                    float fog_amount = fog_density * distance_factor;

                    // Blend original pixel with fog color
                    pixel[0] = static_cast<uchar>(pixel[0] * (1.0f - fog_amount) + fog_color[0] * fog_amount);
                    pixel[1] = static_cast<uchar>(pixel[1] * (1.0f - fog_amount) + fog_color[1] * fog_amount);
                    pixel[2] = static_cast<uchar>(pixel[2] * (1.0f - fog_amount) + fog_color[2] * fog_amount);
                }
            }
        }

        return fogged_image;
    }

    cv::Mat applyTemperatureEffects(const cv::Mat& image, float temperature, const SensorParameters& sensor) {
        cv::Mat temp_effected = image.clone();

        // Temperature affects sensor noise and dark current
        float temp_factor = (temperature - 20.0f) / 10.0f;  // Relative to room temp

        if (std::abs(temp_factor) > 0.1f) {
            // Increase noise at higher temperatures
            float noise_multiplier = 1.0f + 0.1f * std::abs(temp_factor);

            cv::Mat noise = cv::Mat::zeros(image.size(), image.type());
            cv::randn(noise, cv::Scalar::all(0), cv::Scalar::all(5.0 * noise_multiplier));
            cv::add(temp_effected, noise, temp_effected);

            // Thermal effects on image
            if (temp_factor > 0) {
                // High temperature: slight color shift toward red
                for (int y = 0; y < temp_effected.rows; ++y) {
                    for (int x = 0; x < temp_effected.cols; ++x) {
                        cv::Vec3b& pixel = temp_effected.at<cv::Vec3b>(y, x);
                        pixel[2] = std::min(255, pixel[2] + static_cast<int>(temp_factor * 5));  // Red channel increase
                    }
                }
            }
        }

        return temp_effected;
    }

    cv::Mat applyHumidityEffects(const cv::Mat& image, float humidity) {
        cv::Mat humid_image = image.clone();

        if (humidity > 70.0f) {
            // High humidity causes slight image softening and contrast reduction
            float softening_factor = (humidity - 70.0f) / 30.0f;  // 0 to 1 for 70-100% humidity

            // Apply slight Gaussian blur
            float blur_sigma = 0.5f * softening_factor;
            if (blur_sigma > 0.1f) {
                cv::GaussianBlur(humid_image, humid_image, cv::Size(0, 0), blur_sigma);
            }

            // Reduce contrast
            float contrast_reduction = 0.1f * softening_factor;
            for (int y = 0; y < humid_image.rows; ++y) {
                for (int x = 0; x < humid_image.cols; ++x) {
                    cv::Vec3b& pixel = humid_image.at<cv::Vec3b>(y, x);
                    pixel[0] = static_cast<uchar>(pixel[0] * (1.0f - contrast_reduction) + 128 * contrast_reduction);
                    pixel[1] = static_cast<uchar>(pixel[1] * (1.0f - contrast_reduction) + 128 * contrast_reduction);
                    pixel[2] = static_cast<uchar>(pixel[2] * (1.0f - contrast_reduction) + 128 * contrast_reduction);
                }
            }
        }

        return humid_image;
    }

    cv::Mat addHailImpact(const cv::Mat& image, float hail_rate) {
        cv::Mat impacted_image = image.clone();

        // Hail creates random bright spots and potential damage patterns
        int num_impacts = static_cast<int>(hail_rate * 0.02);  // Simplified scaling

        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<int> x_dist(0, image.cols - 1);
        std::uniform_int_distribution<int> y_dist(0, image.rows - 1);

        for (int i = 0; i < num_impacts; ++i) {
            int x = x_dist(gen);
            int y = y_dist(gen);

            // Hail impact creates bright spot with potential "crack" pattern
            cv::circle(impacted_image, cv::Point(x, y), 3, cv::Scalar(255, 255, 255), -1);
        }

        return impacted_image;
    }
};
```

## Behavioral Realism

### System Delays and Latencies

Modeling realistic system delays:

```cpp
class DelayModel {
public:
    struct SystemDelays {
        float sensor_processing_delay;     // Time for sensor to process data
        float communication_delay;         // Network/communication delay
        float computation_delay;           // Time for algorithm computation
        float actuator_response_delay;     // Time for actuator to respond
        float control_loop_delay;          // Total control loop delay
    };

    template<typename T>
    class DelayBuffer {
    private:
        std::queue<std::pair<double, T>> buffer_;  // timestamp, value
        double delay_duration_;
        mutable std::mutex buffer_mutex_;

    public:
        DelayBuffer(double delay_duration) : delay_duration_(delay_duration) {}

        void addValue(const T& value, double current_time) {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            buffer_.push({current_time + delay_duration_, value});
        }

        bool getValue(T& output, double current_time) {
            std::lock_guard<std::mutex> lock(buffer_mutex_);

            // Remove outdated values
            while (!buffer_.empty() && buffer_.front().first < current_time) {
                buffer_.pop();
            }

            if (!buffer_.empty() && buffer_.front().first <= current_time) {
                output = buffer_.front().second;
                buffer_.pop();
                return true;
            }

            return false;  // No delayed value available yet
        }

        void clear() {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            while (!buffer_.empty()) {
                buffer_.pop();
            }
        }
    };

    class RealisticSensor {
    private:
        DelayBuffer<SensorReading> processing_delay_buffer_;
        DelayBuffer<SensorReading> communication_delay_buffer_;
        std::function<double()> noise_generator_;
        std::function<double()> drift_generator_;
        double last_reading_time_;
        SensorReading last_output_;

    public:
        struct SensorReading {
            double value;
            double timestamp;
            double noise;
            double drift;
            bool valid;
        };

        RealisticSensor(double processing_delay, double communication_delay)
            : processing_delay_buffer_(processing_delay),
              communication_delay_buffer_(communication_delay),
              last_reading_time_(0.0) {

            // Initialize noise and drift generators
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::normal_distribution<double> noise_dist(0.0, 0.01);  // 1% noise
            std::normal_distribution<double> drift_dist(0.0, 0.001); // Slow drift

            noise_generator_ = [noise_dist, gen]() mutable { return noise_dist(gen); };
            drift_generator_ = [drift_dist, gen]() mutable { return drift_dist(gen); };
        }

        SensorReading readSensor(double true_value, double current_time) {
            SensorReading raw_reading;
            raw_reading.value = true_value;
            raw_reading.timestamp = current_time;
            raw_reading.noise = noise_generator_();
            raw_reading.drift = drift_generator_();
            raw_reading.valid = true;

            // Apply processing delay
            processing_delay_buffer_.addValue(raw_reading, current_time);

            // Retrieve value after processing delay
            SensorReading processed_reading;
            if (processing_delay_buffer_.getValue(processed_reading, current_time)) {
                // Add processing-specific effects
                processed_reading.value += processed_reading.noise + processed_reading.drift;

                // Apply communication delay
                communication_delay_buffer_.addValue(processed_reading, current_time);

                // Retrieve final value after communication delay
                SensorReading final_reading;
                if (communication_delay_buffer_.getValue(final_reading, current_time)) {
                    last_output_ = final_reading;
                    return final_reading;
                }
            }

            // Return last valid output if delayed value not ready
            return last_output_;
        }
    };

    class RealisticActuator {
    private:
        DelayBuffer<double> command_delay_buffer_;
        double current_position_;
        double target_position_;
        double max_velocity_;
        double max_acceleration_;
        double dead_zone_;
        std::function<double()> noise_generator_;
        std::function<double()> friction_generator_;

    public:
        RealisticActuator(double command_delay, double max_vel, double max_acc, double dead_zone)
            : command_delay_buffer_(command_delay),
              current_position_(0.0),
              target_position_(0.0),
              max_velocity_(max_vel),
              max_acceleration_(max_acc),
              dead_zone_(dead_zone) {

            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::normal_distribution<double> noise_dist(0.0, 0.005);  // 0.5% noise
            std::normal_distribution<double> friction_dist(0.0, 0.01); // Friction variation

            noise_generator_ = [noise_dist, gen]() mutable { return noise_dist(gen); };
            friction_generator_ = [friction_dist, gen]() mutable { return friction_dist(gen); };
        }

        void setCommand(double command, double current_time) {
            // Apply command delay
            command_delay_buffer_.addValue(command, current_time);
        }

        double update(double dt, double current_time) {
            // Check for delayed command
            double delayed_command;
            if (command_delay_buffer_.getValue(delayed_command, current_time)) {
                target_position_ = delayed_command;
            }

            // Apply dead zone
            double error = target_position_ - current_position_;
            if (std::abs(error) < dead_zone_) {
                error = 0.0;
            } else {
                error = error - std::copysign(dead_zone_, error);
            }

            // Calculate required velocity to reach target
            double required_velocity = error / std::max(dt, 0.001);  // Avoid division by zero
            required_velocity = std::clamp(required_velocity, -max_velocity_, max_velocity_);

            // Apply acceleration limits
            double current_velocity = (current_position_ - prev_position_) / std::max(dt, 0.001);
            double velocity_change = required_velocity - current_velocity;
            velocity_change = std::clamp(velocity_change, -max_acceleration_ * dt, max_acceleration_ * dt);

            double new_velocity = current_velocity + velocity_change;
            new_velocity = std::clamp(new_velocity, -max_velocity_, max_velocity_);

            // Update position
            double new_position = current_position_ + new_velocity * dt;

            // Apply noise and friction effects
            new_position += noise_generator_() + friction_generator_() * dt * std::abs(new_velocity);

            prev_position_ = current_position_;
            current_position_ = new_position;

            return current_position_;
        }

    private:
        double prev_position_ = 0.0;
    };
};
```

### Wear and Degradation Modeling

Simulating sensor degradation over time:

```cpp
class DegradationModel {
public:
    struct ComponentHealth {
        float performance_degradation;  // 0.0 (perfect) to 1.0 (failed)
        float accumulated_wear;         // Wear factor (0 to inf)
        float contamination_level;      // Contamination (0.0 to 1.0)
        float thermal_damage;           // Thermal damage (0.0 to 1.0)
        float vibration_damage;         // Vibration damage (0.0 to 1.0)
        double last_maintenance_time;
        bool requires_maintenance;
    };

    class RealisticSensorWithDegradation {
    private:
        ComponentHealth health_;
        double installation_time_;
        double current_time_;
        double base_noise_level_;
        double base_bias_drift_;
        double contamination_accumulation_rate_;
        double thermal_stress_coefficient_;
        double vibration_sensitivity_;

    public:
        RealisticSensorWithDegradation(double installation_time)
            : installation_time_(installation_time),
              base_noise_level_(0.01),
              base_bias_drift_(1e-6),
              contamination_accumulation_rate_(1e-5),
              thermal_stress_coefficient_(1e-4),
              vibration_sensitivity_(1e-3) {

            health_.performance_degradation = 0.0;
            health_.accumulated_wear = 0.0;
            health_.contamination_level = 0.0;
            health_.thermal_damage = 0.0;
            health_.vibration_damage = 0.0;
            health_.last_maintenance_time = installation_time;
            health_.requires_maintenance = false;
        }

        struct SensorOutput {
            double value;
            double noise;
            double bias_drift;
            double scale_factor_error;
            bool fault_detected;
            ComponentHealth current_health;
        };

        SensorOutput readWithDegradation(double true_value,
                                       double temperature,
                                       double vibration_level,
                                       double contamination_exposure,
                                       double current_time) {

            current_time_ = current_time;
            updateDegradation(temperature, vibration_level, contamination_exposure);

            SensorOutput output;
            output.current_health = health_;

            // Calculate degradation effects
            double noise_multiplier = 1.0 + health_.performance_degradation * 2.0;
            double drift_multiplier = 1.0 + health_.performance_degradation * 5.0;
            double scale_error = health_.contamination_level * 0.01;  // 1% max scale error

            // Apply degradation to sensor characteristics
            output.noise = base_noise_level_ * noise_multiplier;
            output.bias_drift = base_bias_drift_ * drift_multiplier * (current_time - installation_time_);
            output.scale_factor_error = scale_error;

            // Apply degradation effects to measurement
            double degraded_value = true_value * (1.0 + output.scale_factor_error);
            degraded_value += output.bias_drift;

            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::normal_distribution<double> noise_dist(0.0, output.noise);

            output.value = degraded_value + noise_dist(gen);

            // Fault detection based on health level
            output.fault_detected = health_.performance_degradation > 0.8;

            return output;
        }

    private:
        void updateDegradation(double temperature,
                              double vibration_level,
                              double contamination_exposure) {

            double dt = current_time_ - (health_.last_maintenance_time > installation_time_ ?
                                        health_.last_maintenance_time : installation_time_);

            // Update contamination
            health_.contamination_level += contamination_exposure * contamination_accumulation_rate_ * dt;
            health_.contamination_level = std::min(1.0, health_.contamination_level);

            // Update thermal damage
            double thermal_stress = std::max(0.0, std::abs(temperature - 25.0) - 10.0) / 50.0;  // Above 35°C or below 15°C
            health_.thermal_damage += thermal_stress * thermal_stress_coefficient_ * dt;
            health_.thermal_damage = std::min(1.0, health_.thermal_damage);

            // Update vibration damage
            health_.vibration_damage += vibration_level * vibration_sensitivity_ * dt;
            health_.vibration_damage = std::min(1.0, health_.vibration_damage);

            // Update accumulated wear
            double environmental_stress = (thermal_stress + health_.contamination_level + vibration_level) / 3.0;
            health_.accumulated_wear += environmental_stress * 0.001 * dt;  // Wear accumulation rate

            // Update performance degradation
            health_.performance_degradation = std::min(1.0,
                0.1 * health_.contamination_level +  // 10% max from contamination
                0.3 * health_.thermal_damage +      // 30% max from thermal
                0.3 * health_.vibration_damage +    // 30% max from vibration
                0.3 * (1.0 - exp(-health_.accumulated_wear))); // 30% max from accumulated wear

            // Maintenance trigger
            health_.requires_maintenance = health_.performance_degradation > 0.7;
        }
    };

    class SystemHealthMonitor {
    private:
        std::vector<RealisticSensorWithDegradation> sensors_;
        std::vector<ComponentHealth> system_health_history_;

    public:
        void addSensor(std::unique_ptr<RealisticSensorWithDegradation> sensor) {
            // This would be handled differently in practice
        }

        struct SystemHealthReport {
            double overall_reliability;
            std::vector<ComponentHealth> component_health;
            std::vector<std::string> maintenance_recommendations;
            double estimated_time_to_failure;
        };

        SystemHealthReport generateHealthReport(double current_time) {
            SystemHealthReport report;
            report.overall_reliability = 1.0;

            for (auto& sensor : sensors_) {
                // This would iterate through actual sensors
            }

            // Calculate overall system reliability
            // This is a simplified model
            report.overall_reliability = 0.9;  // Placeholder

            // Generate maintenance recommendations
            if (report.overall_reliability < 0.8) {
                report.maintenance_recommendations.push_back("Schedule maintenance soon");
            }
            if (report.overall_reliability < 0.6) {
                report.maintenance_recommendations.push_back("Immediate maintenance required");
            }

            // Estimate time to failure (simplified)
            report.estimated_time_to_failure = 100.0;  // Placeholder

            return report;
        }
    };
};
```

## Validation and Assessment

### Realism Metrics

Quantifying simulation realism:

```cpp
class RealismAssessment {
public:
    struct RealismMetrics {
        double photorealism_score;      // 0-1, higher is more realistic
        double physical_accuracy;       // 0-1, higher is more accurate
        double temporal_coherence;      // 0-1, consistency over time
        double sensor_fidelity;         // 0-1, sensor behavior accuracy
        double environmental_validity;  // 0-1, environment realism
        double overall_realism_score;   // Weighted average
    };

    RealismMetrics assessSimulation(const SimulationData& sim_data,
                                   const RealWorldData& real_data) {

        RealismMetrics metrics;

        // Assess photorealism
        metrics.photorealism_score = assessPhotorealism(sim_data.visual_data, real_data.visual_data);

        // Assess physical accuracy
        metrics.physical_accuracy = assessPhysicalAccuracy(sim_data.physics_data, real_data.physics_data);

        // Assess temporal coherence
        metrics.temporal_coherence = assessTemporalCoherence(sim_data.time_series, real_data.time_series);

        // Assess sensor fidelity
        metrics.sensor_fidelity = assessSensorFidelity(sim_data.sensor_data, real_data.sensor_data);

        // Assess environmental validity
        metrics.environmental_validity = assessEnvironmentalValidity(sim_data.env_data, real_data.env_data);

        // Calculate weighted overall score
        metrics.overall_realism_score =
            0.3 * metrics.photorealism_score +
            0.25 * metrics.physical_accuracy +
            0.15 * metrics.temporal_coherence +
            0.2 * metrics.sensor_fidelity +
            0.1 * metrics.environmental_validity;

        return metrics;
    }

private:
    double assessPhotorealism(const std::vector<cv::Mat>& sim_images,
                             const std::vector<cv::Mat>& real_images) {

        if (sim_images.empty() || real_images.empty()) return 0.0;

        double total_score = 0.0;
        int comparison_count = 0;

        for (size_t i = 0; i < std::min(sim_images.size(), real_images.size()); ++i) {
            // Calculate Structural Similarity Index (SSIM)
            double ssim_score = calculateSSIM(sim_images[i], real_images[i]);

            // Calculate Naturalness Image Quality Evaluator (NIQE) - simplified
            double niqe_sim = calculateNIQE(sim_images[i]);
            double niqe_real = calculateNIQE(real_images[i]);
            double niqe_score = 1.0 - std::abs(niqe_sim - niqe_real) / 10.0;  // Normalize

            // Combine scores
            double frame_score = 0.7 * ssim_score + 0.3 * std::max(0.0, niqe_score);
            total_score += frame_score;
            comparison_count++;
        }

        return comparison_count > 0 ? total_score / comparison_count : 0.0;
    }

    double assessPhysicalAccuracy(const PhysicsData& sim_physics,
                                 const PhysicsData& real_physics) {

        // Compare trajectory accuracy
        double trajectory_error = calculateTrajectoryError(sim_physics.trajectories, real_physics.trajectories);

        // Compare force/torque accuracy
        double force_error = calculateForceError(sim_physics.forces, real_physics.forces);

        // Compare energy conservation
        double energy_error = calculateEnergyError(sim_physics.energies, real_physics.energies);

        // Combine errors (convert to accuracy scores)
        double trajectory_accuracy = std::max(0.0, 1.0 - trajectory_error);
        double force_accuracy = std::max(0.0, 1.0 - force_error);
        double energy_accuracy = std::max(0.0, 1.0 - energy_error);

        return 0.5 * trajectory_accuracy + 0.3 * force_accuracy + 0.2 * energy_accuracy;
    }

    double assessTemporalCoherence(const TimeSeriesData& sim_ts,
                                  const TimeSeriesData& real_ts) {

        // Compare temporal consistency of features
        double temporal_stability = calculateTemporalStability(sim_ts, real_ts);

        // Compare frequency content
        double frequency_similarity = calculateFrequencySimilarity(sim_ts, real_ts);

        // Compare phase relationships
        double phase_coherence = calculatePhaseCoherence(sim_ts, real_ts);

        return 0.4 * temporal_stability + 0.3 * frequency_similarity + 0.3 * phase_coherence;
    }

    double assessSensorFidelity(const SensorData& sim_sensors,
                               const SensorData& real_sensors) {

        // Compare noise characteristics
        double noise_similarity = compareNoiseCharacteristics(sim_sensors, real_sensors);

        // Compare response time
        double response_time_match = compareResponseTimes(sim_sensors, real_sensors);

        // Compare accuracy under various conditions
        double condition_accuracy = compareConditionAccuracy(sim_sensors, real_sensors);

        return 0.4 * noise_similarity + 0.3 * response_time_match + 0.3 * condition_accuracy;
    }

    double assessEnvironmentalValidity(const EnvironmentData& sim_env,
                                      const EnvironmentData& real_env) {

        // Compare lighting conditions
        double lighting_similarity = compareLighting(sim_env.lighting, real_env.lighting);

        // Compare material properties
        double material_similarity = compareMaterials(sim_env.materials, real_env.materials);

        // Compare environmental dynamics
        double dynamics_similarity = compareDynamics(sim_env.dynamics, real_env.dynamics);

        return 0.4 * lighting_similarity + 0.3 * material_similarity + 0.3 * dynamics_similarity;
    }

    // Helper functions for specific assessments
    double calculateSSIM(const cv::Mat& img1, const cv::Mat& img2) {
        if (img1.size() != img2.size()) return 0.0;

        cv::Mat gray1, gray2;
        cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);

        // Calculate mean and variance
        cv::Scalar mu1, mu2, var1, var2, covar;
        cv::meanStdDev(gray1, mu1, var1);
        cv::meanStdDev(gray2, mu2, var2);

        double mu1_val = mu1[0];
        double mu2_val = mu2[0];
        double var1_val = var1[0] * var1[0];
        double var2_val = var2[0] * var2[0];

        // SSIM constants
        double c1 = (0.01 * 255) * (0.01 * 255);
        double c2 = (0.03 * 255) * (0.03 * 255);

        double numerator = (2 * mu1_val * mu2_val + c1) * (2 * sqrt(var1_val) * sqrt(var2_val) + c2);
        double denominator = (mu1_val*mu1_val + mu2_val*mu2_val + c1) * (var1_val + var2_val + c2);

        return numerator / denominator;
    }

    double calculateNIQE(const cv::Mat& image) {
        // Simplified NIQE calculation (full implementation is complex)
        // This is a placeholder that calculates a basic quality metric

        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // Calculate basic statistics
        cv::Scalar mean, std_dev;
        cv::meanStdDev(gray, mean, std_dev);

        // Simplified NIQE-like score (lower is better)
        double mean_val = mean[0];
        double std_val = std_dev[0];

        // Combine statistics into a quality score
        // This is not a true NIQE implementation but serves the purpose
        return std::abs(mean_val - 128.0) / 128.0 + std::abs(std_val - 64.0) / 64.0;
    }

    double calculateTrajectoryError(const std::vector<TrajectoryPoint>& sim_traj,
                                   const std::vector<TrajectoryPoint>& real_traj) {

        if (sim_traj.size() != real_traj.size() || sim_traj.empty()) return 1.0;

        double total_error = 0.0;
        for (size_t i = 0; i < sim_traj.size(); ++i) {
            double pos_error = cv::norm(cv::Point3d(sim_traj[i].position - real_traj[i].position));
            total_error += pos_error;
        }

        return total_error / sim_traj.size();  // Average positional error
    }

    double calculateForceError(const std::vector<ForceData>& sim_forces,
                              const std::vector<ForceData>& real_forces) {

        if (sim_forces.size() != real_forces.size() || sim_forces.empty()) return 1.0;

        double total_error = 0.0;
        for (size_t i = 0; i < sim_forces.size(); ++i) {
            double force_error = cv::norm(cv::Point3d(sim_forces[i].force - real_forces[i].force));
            total_error += force_error;
        }

        return total_error / sim_forces.size();  // Average force error
    }

    double calculateEnergyError(const std::vector<EnergyData>& sim_energy,
                               const std::vector<EnergyData>& real_energy) {

        if (sim_energy.size() != real_energy.size() || sim_energy.empty()) return 1.0;

        double total_error = 0.0;
        for (size_t i = 0; i < sim_energy.size(); ++i) {
            double energy_error = std::abs(sim_energy[i].total_energy - real_energy[i].total_energy);
            total_error += energy_error;
        }

        return total_error / sim_energy.size();  // Average energy error
    }

    double calculateTemporalStability(const TimeSeriesData& sim_ts, const TimeSeriesData& real_ts) {
        // Calculate stability of features over time
        // This would involve analyzing time-series consistency
        return 0.8;  // Placeholder
    }

    double calculateFrequencySimilarity(const TimeSeriesData& sim_ts, const TimeSeriesData& real_ts) {
        // Compare frequency content of time series
        // This would involve FFT analysis
        return 0.7;  // Placeholder
    }

    double calculatePhaseCoherence(const TimeSeriesData& sim_ts, const TimeSeriesData& real_ts) {
        // Compare phase relationships in time series
        return 0.75; // Placeholder
    }

    double compareNoiseCharacteristics(const SensorData& sim_sensors, const SensorData& real_sensors) {
        // Compare statistical properties of sensor noise
        return 0.85; // Placeholder
    }

    double compareResponseTimes(const SensorData& sim_sensors, const SensorData& real_sensors) {
        // Compare sensor response times
        return 0.8;  // Placeholder
    }

    double compareConditionAccuracy(const SensorData& sim_sensors, const SensorData& real_sensors) {
        // Compare accuracy under various conditions
        return 0.75; // Placeholder
    }

    double compareLighting(const LightingData& sim_light, const LightingData& real_light) {
        // Compare lighting conditions and effects
        return 0.9;  // Placeholder
    }

    double compareMaterials(const MaterialData& sim_mat, const MaterialData& real_mat) {
        // Compare material properties and behaviors
        return 0.85; // Placeholder
    }

    double compareDynamics(const DynamicsData& sim_dyn, const DynamicsData& real_dyn) {
        // Compare environmental dynamics
        return 0.8;  // Placeholder
    }

    struct SimulationData {
        std::vector<cv::Mat> visual_data;
        PhysicsData physics_data;
        TimeSeriesData time_series;
        SensorData sensor_data;
        EnvironmentData env_data;
    };

    struct RealWorldData {
        std::vector<cv::Mat> visual_data;
        PhysicsData physics_data;
        TimeSeriesData time_series;
        SensorData sensor_data;
        EnvironmentData env_data;
    };

    struct PhysicsData {
        std::vector<TrajectoryPoint> trajectories;
        std::vector<ForceData> forces;
        std::vector<EnergyData> energies;
    };

    struct TrajectoryPoint {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        double time;
    };

    struct ForceData {
        Eigen::Vector3d force;
        Eigen::Vector3d torque;
        double time;
    };

    struct EnergyData {
        double kinetic_energy;
        double potential_energy;
        double total_energy;
        double time;
    };

    struct TimeSeriesData {
        std::vector<double> values;
        std::vector<double> timestamps;
    };

    struct SensorData {
        std::vector<SensorReading> readings;
        std::vector<double> timestamps;
    };

    struct EnvironmentData {
        LightingData lighting;
        MaterialData materials;
        DynamicsData dynamics;
    };

    struct LightingData {
        std::vector<LightSource> lights;
        double ambient_intensity;
    };

    struct MaterialData {
        std::vector<MaterialProperties> materials;
    };

    struct DynamicsData {
        std::vector<DynamicEffect> effects;
    };

    struct SensorReading {
        double value;
        double noise;
        double timestamp;
    };

    struct MaterialProperties {
        double density;
        double elasticity;
        double friction_coefficient;
        double thermal_conductivity;
    };

    struct DynamicEffect {
        double magnitude;
        double frequency;
        double duration;
    };
};
```

## Domain Randomization

### Technique for Improving Transfer Learning

Domain randomization to bridge the reality gap:

```cpp
class DomainRandomizer {
public:
    struct RandomizationParameters {
        // Visual domain randomization
        float texture_randomization;      // 0.0-1.0, how much to vary textures
        float lighting_randomization;     // 0.0-1.0, how much to vary lighting
        float color_randomization;        // 0.0-1.0, how much to vary colors
        float shadow_randomization;       // 0.0-1.0, how much to vary shadows
        float camera_noise_randomization; // 0.0-1.0, how much to vary noise

        // Physical domain randomization
        float friction_randomization;     // 0.0-1.0, how much to vary friction
        float mass_randomization;         // 0.0-1.0, how much to vary masses
        float damping_randomization;      // 0.0-1.0, how much to vary damping
        float restitution_randomization;  // 0.0-1.0, how much to vary bounciness

        // Environmental domain randomization
        float object_placement_randomization; // 0.0-1.0, how much to vary object positions
        float background_randomization;       // 0.0-1.0, how much to vary backgrounds
        float weather_randomization;          // 0.0-1.0, how much to vary weather
    };

    class RandomizedSceneGenerator {
    private:
        RandomizationParameters params_;
        std::random_device rd_;
        std::mt19937 gen_;
        std::uniform_real_distribution<float> uniform_dist_;

    public:
        RandomizedSceneGenerator(const RandomizationParameters& params)
            : params_(params), gen_(rd_()), uniform_dist_(0.0f, 1.0f) {}

        Scene generateRandomizedScene(const Scene& base_scene) {
            Scene randomized_scene = base_scene;

            // Apply visual randomization
            applyVisualRandomization(randomized_scene);

            // Apply physical randomization
            applyPhysicalRandomization(randomized_scene);

            // Apply environmental randomization
            applyEnvironmentalRandomization(randomized_scene);

            return randomized_scene;
        }

    private:
        void applyVisualRandomization(Scene& scene) {
            // Randomize textures and materials
            if (params_.texture_randomization > 0) {
                randomizeTextures(scene);
            }

            // Randomize lighting
            if (params_.lighting_randomization > 0) {
                randomizeLighting(scene);
            }

            // Randomize colors
            if (params_.color_randomization > 0) {
                randomizeColors(scene);
            }

            // Randomize shadows
            if (params_.shadow_randomization > 0) {
                randomizeShadows(scene);
            }

            // Randomize camera parameters
            if (params_.camera_noise_randomization > 0) {
                randomizeCameraParameters(scene);
            }
        }

        void randomizeTextures(Scene& scene) {
            for (auto& object : scene.objects) {
                if (uniform_dist_(gen_) < params_.texture_randomization) {
                    // Change texture with random variation
                    float variation = (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.texture_randomization;

                    // Modify texture properties
                    object.material.texture_scale += variation * 0.5f;
                    object.material.roughness += variation * 0.3f;
                    object.material.metallic += variation * 0.1f;

                    // Ensure values stay in valid range
                    object.material.texture_scale = std::max(0.1f, object.material.texture_scale);
                    object.material.roughness = std::clamp(object.material.roughness, 0.0f, 1.0f);
                    object.material.metallic = std::clamp(object.material.metallic, 0.0f, 1.0f);
                }
            }
        }

        void randomizeLighting(Scene& scene) {
            for (auto& light : scene.lights) {
                if (uniform_dist_(gen_) < params_.lighting_randomization) {
                    // Randomize light properties
                    float intensity_variation = 1.0f + (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.lighting_randomization;
                    light.intensity *= std::max(0.1f, intensity_variation);

                    // Randomize light color
                    if (uniform_dist_(gen_) < 0.5f * params_.lighting_randomization) {
                        light.color.x() += (uniform_dist_(gen_) - 0.5f) * params_.lighting_randomization;
                        light.color.y() += (uniform_dist_(gen_) - 0.5f) * params_.lighting_randomization;
                        light.color.z() += (uniform_dist_(gen_) - 0.5f) * params_.lighting_randomization;

                        // Clamp to valid range
                        light.color.x() = std::clamp(light.color.x(), 0.0f, 1.0f);
                        light.color.y() = std::clamp(light.color.y(), 0.0f, 1.0f);
                        light.color.z() = std::clamp(light.color.z(), 0.0f, 1.0f);
                    }

                    // Randomize light position/direction for dynamic lights
                    if (light.type == LightType::DIRECTIONAL && uniform_dist_(gen_) < 0.3f * params_.lighting_randomization) {
                        Eigen::Vector3f perturbation(
                            (uniform_dist_(gen_) - 0.5f) * 0.2f * params_.lighting_randomization,
                            (uniform_dist_(gen_) - 0.5f) * 0.2f * params_.lighting_randomization,
                            (uniform_dist_(gen_) - 0.5f) * 0.2f * params_.lighting_randomization
                        );
                        light.direction = (light.direction + perturbation).normalized();
                    }
                }
            }
        }

        void randomizeColors(Scene& scene) {
            for (auto& object : scene.objects) {
                if (uniform_dist_(gen_) < params_.color_randomization) {
                    // Add random color variation
                    float hue_variation = (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.color_randomization * 0.2f;
                    float saturation_variation = (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.color_randomization * 0.1f;
                    float value_variation = (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.color_randomization * 0.1f;

                    // Convert RGB to HSV, apply variation, convert back to RGB
                    cv::Vec3f rgb(object.material.albedo.x(), object.material.albedo.y(), object.material.albedo.z());
                    cv::Vec3f hsv = rgbToHsv(rgb);

                    hsv[0] = std::clamp(hsv[0] + hue_variation, 0.0f, 1.0f);
                    hsv[1] = std::clamp(hsv[1] + saturation_variation, 0.0f, 1.0f);
                    hsv[2] = std::clamp(hsv[2] + value_variation, 0.0f, 1.0f);

                    cv::Vec3f new_rgb = hsvToRgb(hsv);
                    object.material.albedo = Eigen::Vector3f(new_rgb[0], new_rgb[1], new_rgb[2]);
                }
            }
        }

        void randomizeShadows(Scene& scene) {
            // In a real implementation, this would modify shadow properties
            // For now, we'll just note that shadow randomization is enabled
            scene.shadow_randomization_enabled = params_.shadow_randomization > 0;
        }

        void randomizeCameraParameters(Scene& scene) {
            // Apply randomization to camera noise and other parameters
            for (auto& camera : scene.cameras) {
                if (uniform_dist_(gen_) < params_.camera_noise_randomization) {
                    // Randomize noise parameters
                    camera.noise_params.readout_noise *= (1.0f + (uniform_dist_(gen_) - 0.5f) * params_.camera_noise_randomization);
                    camera.noise_params.photon_shot_noise *= (1.0f + (uniform_dist_(gen_) - 0.5f) * params_.camera_noise_randomization);
                    camera.noise_params.dark_current *= (1.0f + (uniform_dist_(gen_) - 0.5f) * params_.camera_noise_randomization);
                }
            }
        }

        void applyPhysicalRandomization(Scene& scene) {
            // Randomize friction coefficients
            if (params_.friction_randomization > 0) {
                for (auto& object : scene.objects) {
                    if (uniform_dist_(gen_) < params_.friction_randomization) {
                        float friction_variation = (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.friction_randomization;
                        object.material.friction_coefficient = std::max(0.01f, object.material.friction_coefficient * (1.0f + friction_variation));
                    }
                }
            }

            // Randomize masses
            if (params_.mass_randomization > 0) {
                for (auto& object : scene.objects) {
                    if (uniform_dist_(gen_) < params_.mass_randomization) {
                        float mass_variation = (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.mass_randomization;
                        object.mass = std::max(0.001f, object.mass * (1.0f + mass_variation));
                    }
                }
            }

            // Randomize damping
            if (params_.damping_randomization > 0) {
                for (auto& object : scene.objects) {
                    if (uniform_dist_(gen_) < params_.damping_randomization) {
                        float damping_variation = (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.damping_randomization;
                        object.linear_damping = std::max(0.0f, object.linear_damping * (1.0f + damping_variation));
                        object.angular_damping = std::max(0.0f, object.angular_damping * (1.0f + damping_variation));
                    }
                }
            }

            // Randomize restitution (bounciness)
            if (params_.restitution_randomization > 0) {
                for (auto& object : scene.objects) {
                    if (uniform_dist_(gen_) < params_.restitution_randomization) {
                        float restitution_variation = (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.restitution_randomization;
                        object.material.restitution = std::clamp(object.material.restitution * (1.0f + restitution_variation), 0.0f, 1.0f);
                    }
                }
            }
        }

        void applyEnvironmentalRandomization(Scene& scene) {
            // Randomize object placements
            if (params_.object_placement_randomization > 0) {
                randomizeObjectPlacements(scene);
            }

            // Randomize background
            if (params_.background_randomization > 0) {
                randomizeBackground(scene);
            }

            // Randomize weather (if applicable)
            if (params_.weather_randomization > 0) {
                randomizeWeather(scene);
            }
        }

        void randomizeObjectPlacements(Scene& scene) {
            for (auto& object : scene.objects) {
                if (uniform_dist_(gen_) < params_.object_placement_randomization) {
                    // Add random displacement
                    Eigen::Vector3f displacement(
                        (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.object_placement_randomization,
                        (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.object_placement_randomization,
                        (uniform_dist_(gen_) - 0.5f) * 2.0f * params_.object_placement_randomization
                    );

                    object.position += displacement;
                }
            }
        }

        void randomizeBackground(Scene& scene) {
            if (uniform_dist_(gen_) < params_.background_randomization) {
                // Change background texture, color, or environment map
                scene.background_type = static_cast<BackgroundType>(
                    static_cast<int>(uniform_dist_(gen_) * 5) % 5  // 5 background types
                );

                // Randomize background color
                scene.background_color = Eigen::Vector3f(
                    uniform_dist_(gen_) * 0.5f + 0.2f,  // Range: 0.2 - 0.7
                    uniform_dist_(gen_) * 0.5f + 0.2f,
                    uniform_dist_(gen_) * 0.5f + 0.2f
                );
            }
        }

        void randomizeWeather(Scene& scene) {
            if (uniform_dist_(gen_) < params_.weather_randomization) {
                // Apply weather effects
                scene.weather_condition = static_cast<WeatherCondition>(
                    static_cast<int>(uniform_dist_(gen_) * 4) % 4  // 4 weather types
                );

                // Adjust environmental parameters based on weather
                switch (scene.weather_condition) {
                    case WeatherCondition::FOGGY:
                        scene.atmospheric_density *= 1.2f;
                        scene.visibility *= 0.5f;
                        break;
                    case WeatherCondition::RAINY:
                        scene.atmospheric_density *= 1.1f;
                        scene.luminosity *= 0.7f;
                        break;
                    case WeatherCondition::SNOWY:
                        scene.atmospheric_density *= 1.05f;
                        scene.luminosity *= 0.8f;
                        break;
                    case WeatherCondition::CLEAR:
                    default:
                        // No adjustment needed
                        break;
                }
            }
        }

        cv::Vec3f rgbToHsv(const cv::Vec3f& rgb) {
            float r = rgb[0], g = rgb[1], b = rgb[2];
            float h, s, v;

            float max_val = std::max({r, g, b});
            float min_val = std::min({r, g, b});
            float delta = max_val - min_val;

            v = max_val;
            if (delta < 1e-6) {
                h = 0;
                s = 0;
            } else {
                s = delta / max_val;
                if (max_val == r) {
                    h = (g - b) / delta + (g < b ? 6 : 0);
                } else if (max_val == g) {
                    h = (b - r) / delta + 2;
                } else {
                    h = (r - g) / delta + 4;
                }
                h /= 6;
            }

            return cv::Vec3f(h, s, v);
        }

        cv::Vec3f hsvToRgb(const cv::Vec3f& hsv) {
            float h = hsv[0], s = hsv[1], v = hsv[2];

            float c = v * s;
            float x = c * (1 - std::abs(std::fmod(h * 6, 2) - 1));
            float m = v - c;

            float r, g, b;
            if (h < 1.0/6) {
                r = c; g = x; b = 0;
            } else if (h < 2.0/6) {
                r = x; g = c; b = 0;
            } else if (h < 3.0/6) {
                r = 0; g = c; b = x;
            } else if (h < 4.0/6) {
                r = 0; g = x; b = c;
            } else if (h < 5.0/6) {
                r = x; g = 0; b = c;
            } else {
                r = c; g = 0; b = x;
            }

            return cv::Vec3f(r + m, g + m, b + m);
        }

        enum class BackgroundType { TEXTURED, SOLID_COLOR, GRADIENT, SKY_BOX, PROCEDURAL };
        enum class WeatherCondition { CLEAR, FOGGY, RAINY, SNOWY, STORMY };
    };

    struct Scene {
        std::vector<SceneObject> objects;
        std::vector<LightSource> lights;
        std::vector<Camera> cameras;
        BackgroundType background_type;
        Eigen::Vector3f background_color;
        WeatherCondition weather_condition;
        float atmospheric_density;
        float visibility;
        float luminosity;
        bool shadow_randomization_enabled;

        struct SceneObject {
            Eigen::Vector3f position;
            Eigen::Quaternionf orientation;
            float mass;
            float linear_damping;
            float angular_damping;
            Material material;
        };

        struct Material {
            Eigen::Vector3f albedo;
            float roughness;
            float metallic;
            float texture_scale;
            float friction_coefficient;
            float restitution;
        };

        struct LightSource {
            enum Type { DIRECTIONAL, POINT, SPOT } type;
            Eigen::Vector3f position;
            Eigen::Vector3f direction;
            Eigen::Vector3f color;
            float intensity;
        };

        struct Camera {
            struct NoiseParameters {
                float readout_noise;
                float photon_shot_noise;
                float dark_current;
            } noise_params;
        };
    };
};
```

## Best Practices for Simulation Realism

### Validation Strategies

Comprehensive validation approach:

```cpp
class RealismValidationFramework {
public:
    struct ValidationResults {
        double simulation_to_real_correlation;  // Correlation between sim and real
        double transfer_success_rate;           // Success rate when transferring to real robot
        std::vector<double> domain_gap_metrics; // Various gap metrics
        double confidence_interval;             // Confidence in results
        std::string recommendations;
    };

    ValidationResults validateRealism(const std::vector<TestScenario>& scenarios) {
        ValidationResults results;

        std::vector<double> correlations;
        std::vector<double> success_rates;

        for (const auto& scenario : scenarios) {
            // Run simulation
            auto sim_result = runSimulation(scenario);

            // Run real-world experiment
            auto real_result = runRealExperiment(scenario);

            // Calculate correlation
            double correlation = calculateCorrelation(sim_result, real_result);
            correlations.push_back(correlation);

            // Calculate transfer success rate
            double success_rate = calculateTransferSuccess(scenario, sim_result, real_result);
            success_rates.push_back(success_rate);
        }

        // Aggregate results
        results.simulation_to_real_correlation = calculateMean(correlations);
        results.transfer_success_rate = calculateMean(success_rates);
        results.confidence_interval = calculateConfidenceInterval(correlations, success_rates);

        // Generate recommendations
        results.recommendations = generateRecommendations(results);

        return results;
    }

private:
    struct TestScenario {
        std::string name;
        RobotConfiguration robot_config;
        EnvironmentConfiguration env_config;
        TaskSpecification task_spec;
        std::vector<SensorConfiguration> sensor_configs;
    };

    struct RobotConfiguration {
        std::string model;
        std::vector<JointProperties> joints;
        std::vector<LinkProperties> links;
        std::vector<ActuatorProperties> actuators;
    };

    struct EnvironmentConfiguration {
        std::string scene_description;
        PhysicsProperties physics_props;
        LightingProperties lighting_props;
        WeatherProperties weather_props;
    };

    struct TaskSpecification {
        std::string task_type;  // navigation, manipulation, etc.
        std::vector<TaskParameter> parameters;
        std::vector<SuccessCriteria> success_criteria;
    };

    struct TaskParameter {
        std::string name;
        double value;
        double tolerance;
    };

    struct SuccessCriteria {
        std::string metric;
        double threshold;
        bool is_greater_better;
    };

    struct JointProperties {
        std::string name;
        double min_position;
        double max_position;
        double max_velocity;
        double max_effort;
        double damping;
        double friction;
    };

    struct LinkProperties {
        std::string name;
        double mass;
        Eigen::Vector3d com;
        Eigen::Matrix3d inertia;
        std::string visual_mesh;
        std::string collision_mesh;
    };

    struct ActuatorProperties {
        std::string joint_name;
        double gear_ratio;
        double efficiency;
        double max_torque;
        double torque_constant;
        double thermal_time_constant;
    };

    struct PhysicsProperties {
        double gravity;
        double time_step;
        double solver_iterations;
        double contact_erp;
        double contact_cfm;
    };

    struct LightingProperties {
        std::vector<LightSource> lights;
        double ambient_intensity;
        double shadow_softness;
    };

    struct WeatherProperties {
        double temperature;
        double humidity;
        double wind_speed;
        double precipitation_rate;
    };

    struct SensorConfiguration {
        std::string sensor_type;
        std::string frame_id;
        double update_rate;
        SensorNoiseProperties noise_properties;
    };

    struct SensorNoiseProperties {
        double gaussian_noise_std;
        double bias_instability;
        double random_walk;
        double scale_factor_error;
        double non_linearity;
    };

    struct SimulationResult {
        std::vector<TrajectoryPoint> trajectory;
        std::vector<SensorData> sensor_readings;
        std::vector<ControlInput> control_inputs;
        double execution_time;
        bool task_completed;
        std::vector<double> performance_metrics;
    };

    struct RealWorldResult {
        std::vector<TrajectoryPoint> trajectory;
        std::vector<SensorData> sensor_readings;
        std::vector<ControlInput> control_inputs;
        double execution_time;
        bool task_completed;
        std::vector<double> performance_metrics;
    };

    struct ControlInput {
        double timestamp;
        std::vector<double> joint_commands;
        std::vector<double> cartesian_commands;
        std::vector<double> other_commands;
    };

    SimulationResult runSimulation(const TestScenario& scenario) {
        // Implementation would run the simulation with given scenario
        SimulationResult result;
        // This is a placeholder - in reality, this would call the simulation engine
        return result;
    }

    RealWorldResult runRealExperiment(const TestScenario& scenario) {
        // Implementation would run the real-world experiment
        RealWorldResult result;
        // This is a placeholder - in reality, this would control the real robot
        return result;
    }

    double calculateCorrelation(const SimulationResult& sim, const RealWorldResult& real) {
        // Calculate correlation between simulation and real-world results
        // This would involve comparing trajectories, sensor readings, etc.

        if (sim.trajectory.size() != real.trajectory.size() || sim.trajectory.empty()) {
            return 0.0;
        }

        // Calculate trajectory correlation
        double position_correlation = 0.0;
        double orientation_correlation = 0.0;

        for (size_t i = 0; i < sim.trajectory.size(); ++i) {
            // Calculate position correlation
            double pos_sim = sim.trajectory[i].position.norm();
            double pos_real = real.trajectory[i].position.norm();

            // Pearson correlation coefficient calculation would go here
            // For simplicity, using a basic distance-based similarity
            double pos_diff = std::abs(pos_sim - pos_real);
            double pos_corr = 1.0 / (1.0 + pos_diff);  // Higher correlation for smaller differences
            position_correlation += pos_corr;

            // Calculate orientation correlation
            double orient_diff = (sim.trajectory[i].orientation.coeffs() - real.trajectory[i].orientation.coeffs()).norm();
            double orient_corr = 1.0 / (1.0 + orient_diff);
            orientation_correlation += orient_corr;
        }

        position_correlation /= sim.trajectory.size();
        orientation_correlation /= sim.trajectory.size();

        // Weighted combination
        return 0.6 * position_correlation + 0.4 * orientation_correlation;
    }

    double calculateTransferSuccess(const TestScenario& scenario,
                                  const SimulationResult& sim_result,
                                  const RealWorldResult& real_result) {
        // Calculate how often the real robot succeeds when simulation predicted success
        if (!sim_result.task_completed) {
            return 0.5;  // Neutral if simulation didn't complete
        }

        return real_result.task_completed ? 1.0 : 0.0;
    }

    double calculateMean(const std::vector<double>& values) {
        if (values.empty()) return 0.0;

        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.size();
    }

    double calculateConfidenceInterval(const std::vector<double>& correlations,
                                     const std::vector<double>& success_rates) {
        // Calculate confidence interval based on variance
        auto [corr_mean, corr_var] = calculateMeanAndVariance(correlations);
        auto [succ_mean, succ_var] = calculateMeanAndVariance(success_rates);

        // Combined variance (weighted average)
        double combined_variance = 0.5 * corr_var + 0.5 * succ_var;
        double std_error = sqrt(combined_variance / (correlations.size() + success_rates.size()));

        // 95% confidence interval (z-score ≈ 1.96)
        return 1.96 * std_error;
    }

    std::pair<double, double> calculateMeanAndVariance(const std::vector<double>& values) {
        if (values.empty()) return {0.0, 0.0};

        double mean = calculateMean(values);
        double variance = 0.0;

        for (double value : values) {
            variance += (value - mean) * (value - mean);
        }
        variance /= values.size();

        return {mean, variance};
    }

    std::string generateRecommendations(const ValidationResults& results) {
        std::string recommendations;

        if (results.simulation_to_real_correlation < 0.7) {
            recommendations += "LOW SIM-TO-REAL CORRELATION: ";
            recommendations += "Consider improving physical accuracy of simulation.\n";
            recommendations += "- Review material properties and friction models\n";
            recommendations += "- Validate sensor noise models against real sensors\n";
            recommendations += "- Check environmental conditions match reality\n";
        }

        if (results.transfer_success_rate < 0.8) {
            recommendations += "LOW TRANSFER SUCCESS RATE: ";
            recommendations += "Reality gap may be too large.\n";
            recommendations += "- Implement domain randomization\n";
            recommendations += "- Add more realistic sensor noise and delays\n";
            recommendations += "- Consider system identification techniques\n";
        }

        if (results.confidence_interval > 0.1) {
            recommendations += "HIGH VARIANCE IN RESULTS: ";
            recommendations += "Need more validation runs for statistical significance.\n";
            recommendations += "- Increase number of test scenarios\n";
            recommendations += "- Run multiple trials per scenario\n";
            recommendations += "- Consider different environmental conditions\n";
        }

        if (recommendations.empty()) {
            recommendations = "SIMULATION REALISM IS ADEQUATE: Current simulation fidelity appears appropriate for intended use.";
        }

        return recommendations;
    }
};
```

## Advanced Techniques

### Neural Rendering for Realism

Using neural networks to enhance visual realism:

```cpp
class NeuralRenderingEnhancer {
public:
    struct NeuralRenderParams {
        float realism_boost;              // How much to enhance realism (0.0-1.0)
        float texture_enhancement;       // Enhance texture details
        float lighting_correction;        // Correct lighting inconsistencies
        float atmospheric_enhancement;   // Enhance atmospheric effects
        float temporal_coherence;        // Maintain consistency over time
    };

    cv::Mat enhanceRealism(const cv::Mat& input_image,
                          const NeuralRenderParams& params,
                          const SceneContext& context) {

        cv::Mat enhanced_image = input_image.clone();

        // Apply neural network-based enhancements
        enhanced_image = applyTextureEnhancement(enhanced_image, params.texture_enhancement);
        enhanced_image = applyLightingCorrection(enhanced_image, context.lighting, params.lighting_correction);
        enhanced_image = applyAtmosphericEnhancement(enhanced_image, context.atmosphere, params.atmospheric_enhancement);
        enhanced_image = applyRealismBoost(enhanced_image, params.realism_boost);

        // Maintain temporal coherence
        enhanced_image = applyTemporalSmoothing(enhanced_image, context.previous_frame, params.temporal_coherence);

        return enhanced_image;
    }

private:
    cv::Mat applyTextureEnhancement(const cv::Mat& image, float enhancement_level) {
        if (enhancement_level <= 0.0f) return image;

        cv::Mat enhanced = image.clone();

        // Apply detail enhancement using unsharp masking
        cv::Mat blurred;
        cv::GaussianBlur(enhanced, blurred, cv::Size(0, 0), 1.0);

        cv::Mat detail = enhanced - blurred;
        cv::addWeighted(enhanced, 1.0, detail, enhancement_level, 0, enhanced);

        return enhanced;
    }

    cv::Mat applyLightingCorrection(const cv::Mat& image,
                                   const LightingContext& lighting,
                                   float correction_strength) {
        if (correction_strength <= 0.0f) return image;

        cv::Mat corrected = image.clone();

        // Adjust exposure based on scene lighting
        float exposure_adjustment = (lighting.average_brightness / 128.0f - 1.0f) * correction_strength;

        // Apply exposure adjustment
        cv::Mat lut(1, 256, CV_8UC1);
        for (int i = 0; i < 256; ++i) {
            float corrected_val = i * exp(exposure_adjustment);
            lut.at<uchar>(i) = cv::saturate_cast<uchar>(corrected_val);
        }

        cv::LUT(corrected, lut, corrected);

        // Color temperature correction
        if (lighting.color_temperature != 6500.0f) {  // 6500K is neutral
            float temp_factor = lighting.color_temperature / 6500.0f;
            for (int y = 0; y < corrected.rows; ++y) {
                for (int x = 0; x < corrected.cols; ++x) {
                    cv::Vec3b& pixel = corrected.at<cv::Vec3b>(y, x);

                    // Apply color temperature adjustment (simplified)
                    pixel[0] = cv::saturate_cast<uchar>(pixel[0] * (temp_factor > 1.0f ? 1.0f : temp_factor));  // Blue channel adjustment
                    pixel[2] = cv::saturate_cast<uchar>(pixel[2] * (temp_factor < 1.0f ? 1.0f : 1.0f + (temp_factor - 1.0f) * 0.3f));  // Red channel adjustment
                }
            }
        }

        return corrected;
    }

    cv::Mat applyAtmosphericEnhancement(const cv::Mat& image,
                                      const AtmosphericContext& atmosphere,
                                      float enhancement_level) {
        if (enhancement_level <= 0.0f) return image;

        cv::Mat enhanced = image.clone();

        // Apply atmospheric effects: haze, fog, distance-based color shift
        float fog_density = atmosphere.fog_density * enhancement_level;
        cv::Vec3b fog_color(180, 190, 200);  // Light gray-blue fog

        for (int y = 0; y < enhanced.rows; ++y) {
            for (int x = 0; x < enhanced.cols; ++x) {
                cv::Vec3b& pixel = enhanced.at<cv::Vec3b>(y, x);

                // Distance-based atmospheric effect (simplified: row number represents distance)
                float distance_factor = static_cast<float>(y) / enhanced.rows;
                float fog_amount = fog_density * distance_factor;

                // Blend with fog color
                pixel[0] = cv::saturate_cast<uchar>(pixel[0] * (1.0f - fog_amount) + fog_color[0] * fog_amount);
                pixel[1] = cv::saturate_cast<uchar>(pixel[1] * (1.0f - fog_amount) + fog_color[1] * fog_amount);
                pixel[2] = cv::saturate_cast<uchar>(pixel[2] * (1.0f - fog_amount) + fog_color[2] * fog_amount);
            }
        }

        return enhanced;
    }

    cv::Mat applyRealismBoost(const cv::Mat& image, float boost_level) {
        if (boost_level <= 0.0f) return image;

        cv::Mat boosted = image.clone();

        // Apply subtle saturation increase
        cv::Mat hsv;
        cv::cvtColor(boosted, hsv, cv::COLOR_BGR2HSV);

        for (int y = 0; y < hsv.rows; ++y) {
            for (int x = 0; x < hsv.cols; ++x) {
                cv::Vec3b& pixel = hsv.at<cv::Vec3b>(y, x);
                // Boost saturation
                pixel[1] = cv::saturate_cast<uchar>(pixel[1] * (1.0f + boost_level * 0.2f));
            }
        }

        cv::cvtColor(hsv, boosted, cv::COLOR_HSV2BGR);

        // Add subtle film grain for photographic realism
        if (boost_level > 0.5f) {
            cv::Mat noise(boosted.size(), boosted.type());
            cv::randu(noise, cv::Scalar::all(0), cv::Scalar::all(boost_level * 10));

            cv::add(boosted, noise, boosted);
        }

        return boosted;
    }

    cv::Mat applyTemporalSmoothing(const cv::Mat& current_frame,
                                  const cv::Mat& previous_frame,
                                  float coherence_level) {
        if (coherence_level <= 0.0f || previous_frame.empty()) return current_frame;

        cv::Mat smoothed = current_frame.clone();

        // Apply temporal smoothing to reduce flickering
        if (!previous_frame.empty() && current_frame.size() == previous_frame.size()) {
            cv::addWeighted(current_frame, 1.0 - coherence_level,
                           previous_frame, coherence_level, 0, smoothed);
        }

        return smoothed;
    }

    struct SceneContext {
        LightingContext lighting;
        AtmosphericContext atmosphere;
        cv::Mat previous_frame;
        float time_of_day;
        float weather_condition;
    };

    struct LightingContext {
        float average_brightness;
        float color_temperature;  // in Kelvin
        float shadow_hardness;
        float ambient_light_ratio;
        std::vector<LightSource> light_sources;
    };

    struct AtmosphericContext {
        float fog_density;
        float humidity;
        float temperature;
        float visibility;
        float wind_speed;
    };
};
```

## Challenges and Limitations

### Computational Complexity

Balancing realism with performance:

**Challenges:**
- High-fidelity physics simulation requires significant computational resources
- Realistic rendering can be computationally expensive
- Multiple sensor modalities compound computational requirements
- Real-time constraints limit simulation complexity

**Solutions:**
- **Adaptive Fidelity:** Adjust simulation quality based on importance of scene
- **LOD (Level of Detail):** Reduce fidelity for distant or less important objects
- **Parallel Processing:** Use multi-core and GPU acceleration
- **Caching:** Store and reuse expensive calculations

### Validation Complexity

**Challenges:**
- Ground truth for real-world comparison is often unavailable
- Multiple interacting factors make isolation difficult
- Validation can be expensive and time-consuming
- Statistical significance requires many trials

**Solutions:**
- **Synthetic Ground Truth:** Use known scenarios with known outcomes
- **Proxy Metrics:** Use indirect measures of realism
- **Statistical Validation:** Employ proper experimental design
- **Incremental Validation:** Validate components individually

## Future Trends

### AI-Enhanced Realism

**Neural Radiance Fields (NeRF):**
- Photorealistic scene reconstruction
- Novel view synthesis
- Improved visual fidelity

**Generative Adversarial Networks:**
- Realistic texture synthesis
- Environment generation
- Style transfer for realism

### Physics-Based Simulation

**Advanced Material Modeling:**
- Plastic deformation
- Fracture mechanics
- Multi-scale material behavior

**Multi-Physics Integration:**
- Electromagnetic effects
- Thermal modeling
- Fluid-structure interaction

## Conclusion

Simulation realism is a multifaceted discipline that requires careful attention to physical accuracy, sensory fidelity, environmental authenticity, and behavioral correctness. The goal of achieving simulation-to-reality transfer requires balancing computational efficiency with modeling accuracy while maintaining the ability to validate and assess the quality of the simulation. Modern approaches increasingly rely on domain randomization, neural enhancement techniques, and sophisticated validation frameworks to bridge the reality gap. As robotics systems become more complex and operate in more diverse environments, simulation realism continues to evolve with advanced rendering techniques, AI-enhanced models, and more sophisticated physical modeling to meet the growing demands of robotic system development and validation. The investment in high-fidelity simulation realism significantly reduces development time, costs, and risks while improving the safety and reliability of robotic systems deployed in real-world applications.
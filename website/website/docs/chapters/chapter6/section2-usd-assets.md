---
sidebar_label: 'USD Assets'
title: 'USD Assets'
---

# USD Assets

## Introduction

Universal Scene Description (USD) assets form the foundation of digital content in NVIDIA Omniverse and other advanced simulation environments. USD, developed by Pixar, provides a powerful, scalable framework for describing, composing, and exchanging complex 3D scenes and assets. In robotics and simulation applications, USD assets enable the creation of detailed, physically-accurate models that can be efficiently shared across different tools and platforms. The hierarchical, layered architecture of USD makes it particularly well-suited for complex robotic systems with multiple components, materials, and behaviors. Understanding USD assets is essential for creating high-fidelity simulation environments that accurately represent real-world robotic systems and their operating environments.

## USD Fundamentals

### Core Concepts

USD is built on several core concepts that enable its powerful scene composition capabilities:

**Composition Arcs:**
- **References**: Create instances of reusable assets
- **Payloads**: Lazy-load heavy content until needed
- **Layers**: Stack multiple scene descriptions
- **Variants**: Provide alternative versions of content

**Schema System:**
USD schemas define the structure and semantics of prims (primitives):
- **Typed Prims**: Strongly typed scene objects (Mesh, Camera, Light, etc.)
- **Relationships**: Directed connections between prims
- **Attributes**: Data associated with prims (colors, transforms, etc.)

### USD File Formats

USD supports multiple file formats optimized for different use cases:

**Plain Text (.usda):**
```
#usda 1.0
def Sphere "MySphere"
{
    radius = 1.0
    xformOp:translate = (0, 0, 0)
}
```

**Binary (.usdc):**
- Compact binary format for faster loading
- Preferred for production environments
- Smaller file sizes with faster parsing

**ASCII (.usd):**
- Can be either text or binary depending on content
- Generic extension that supports both formats

### Stage and Prim Architecture

In USD terminology:
- **Stage**: Container for the entire scene
- **Prim**: Individual scene objects (primitives)
- **Attribute**: Data associated with prims
- **Relationship**: Connections between prims

```cpp
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/sphere.h"
#include "pxr/usd/usdGeom/xform.h"
#include "pxr/usd/sdf/path.h"

class USDAssetManager {
private:
    pxr::UsdStageRefPtr stage_;
    std::string asset_path_;

public:
    USDAssetManager(const std::string& asset_path) : asset_path_(asset_path) {
        stage_ = pxr::UsdStage::CreateNew(asset_path);
    }

    pxr::UsdPrim createSphere(const std::string& name,
                             const pxr::GfVec3f& position,
                             float radius) {

        // Create a sphere prim
        pxr::UsdGeomSphere sphere = pxr::UsdGeomSphere::Define(stage_,
                                                              pxr::SdfPath("/" + name));

        // Set attributes
        sphere.GetRadiusAttr().Set(radius);
        sphere.GetXformOp(pxr::UsdGeomTokens->translate).Set(position);

        return sphere.GetPrim();
    }

    pxr::UsdPrim createRobotModel(const std::string& robot_name) {
        // Create robot root prim
        pxr::UsdGeomXform robot_root = pxr::UsdGeomXform::Define(stage_,
                                                                pxr::SdfPath("/" + robot_name));

        // Create robot links as child prims
        createRobotLink(robot_root, "base_link", pxr::GfVec3f(0, 0, 0));
        createRobotLink(robot_root, "shoulder_link", pxr::GfVec3f(0, 0, 0.1));
        createRobotLink(robot_root, "elbow_link", pxr::GfVec3f(0, 0, 0.5));

        return robot_root.GetPrim();
    }

private:
    pxr::UsdPrim createRobotLink(const pxr::UsdGeomXform& parent,
                                const std::string& link_name,
                                const pxr::GfVec3f& offset) {

        std::string full_path = parent.GetPath().GetString() + "/" + link_name;
        pxr::UsdGeomXform link = pxr::UsdGeomXform::Define(stage_, pxr::SdfPath(full_path));

        link.AddTranslateOp().Set(offset);

        // Add physics properties if needed
        addPhysicsProperties(link);

        return link.GetPrim();
    }

    void addPhysicsProperties(const pxr::UsdGeomXform& link) {
        // Add mass properties using PhysX schemas
        // This would typically involve adding Physics APIs to the prim
    }
};
```

## Asset Creation and Management

### Robot Asset Structure

Creating comprehensive robot assets requires organizing components hierarchically:

```
robot_name.usd
├── links/
│   ├── base_link/
│   │   ├── visuals/
│   │   ├── collisions/
│   │   └── physics/
│   ├── shoulder_link/
│   │   ├── visuals/
│   │   ├── collisions/
│   │   └── physics/
│   └── end_effector/
│       ├── visuals/
│       ├── collisions/
│       └── sensors/
├── joints/
│   ├── base_to_shoulder/
│   └── shoulder_to_elbow/
├── materials/
└── variants/
    ├── configurations/
    └── payloads/
```

### Creating Robot Links

```cpp
class RobotLinkCreator {
public:
    struct LinkDefinition {
        std::string name;
        float mass;
        pxr::GfVec3f center_of_mass;
        pxr::GfVec3f moments_of_inertia;  // Ixx, Iyy, Izz
        std::string visual_mesh_path;
        std::string collision_mesh_path;
        std::string material_path;
    };

    pxr::UsdPrim createRobotLink(pxr::UsdStageRefPtr stage,
                                const std::string& parent_path,
                                const LinkDefinition& link_def) {

        std::string link_path = parent_path + "/" + link_def.name;
        pxr::UsdGeomXform link_prim = pxr::UsdGeomXform::Define(stage, pxr::SdfPath(link_path));

        // Create visual representation
        if (!link_def.visual_mesh_path.empty()) {
            createVisualRepresentation(stage, link_path, link_def);
        }

        // Create collision representation
        if (!link_def.collision_mesh_path.empty()) {
            createCollisionRepresentation(stage, link_path, link_def);
        }

        // Add mass properties
        addMassProperties(stage, link_path, link_def);

        return link_prim.GetPrim();
    }

private:
    void createVisualRepresentation(pxr::UsdStageRefPtr stage,
                                   const std::string& link_path,
                                   const LinkDefinition& link_def) {

        std::string visual_path = link_path + "/visual";
        pxr::UsdGeomXform visual_parent = pxr::UsdGeomXform::Define(stage, pxr::SdfPath(visual_path));

        // Add mesh geometry
        pxr::UsdGeomMesh visual_mesh = pxr::UsdGeomMesh::Define(stage, pxr::SdfPath(visual_path + "/geometry"));
        visual_mesh.GetMeshSubdivisionSchemeAttr().Set(pxr::UsdGeomTokens->none);

        // Add material binding
        if (!link_def.material_path.empty()) {
            pxr::UsdShadeMaterial material = pxr::UsdShadeMaterial::Get(stage, pxr::SdfPath(link_def.material_path));
            pxr::UsdShadeMaterialBindingAPI binding_api(visual_mesh);
            binding_api.Bind(material);
        }
    }

    void createCollisionRepresentation(pxr::UsdStageRefPtr stage,
                                      const std::string& link_path,
                                      const LinkDefinition& link_def) {

        std::string collision_path = link_path + "/collision";
        pxr::UsdGeomXform collision_parent = pxr::UsdGeomXform::Define(stage, pxr::SdfPath(collision_path));

        // Add collision mesh
        pxr::UsdGeomMesh collision_mesh = pxr::UsdGeomMesh::Define(stage, pxr::SdfPath(collision_path + "/geometry"));

        // Apply collision-specific properties
        pxr::UsdPhysicsCollisionAPI collision_api = pxr::UsdPhysicsCollisionAPI::Apply(collision_mesh);
        collision_api.GetCollisionEnabledAttr().Set(true);
    }

    void addMassProperties(pxr::UsdStageRefPtr stage,
                          const std::string& link_path,
                          const LinkDefinition& link_def) {

        // Apply mass API
        pxr::UsdPhysicsMassAPI mass_api = pxr::UsdPhysicsMassAPI::Apply(stage->GetPrimAtPath(pxr::SdfPath(link_path)));

        // Set mass
        mass_api.GetMassAttr().Set(link_def.mass);

        // Set center of mass
        mass_api.GetCenterOfMassAttr().Set(link_def.center_of_mass);

        // Set moments of inertia
        pxr::GfVec3f diagonal_inertia(
            link_def.moments_of_inertia[0],  // Ixx
            link_def.moments_of_inertia[1],  // Iyy
            link_def.moments_of_inertia[2]   // Izz
        );
        mass_api.GetDiagonalInertiaAttr().Set(diagonal_inertia);
    }
};
```

### Material Definition and Application

Creating realistic materials for robot assets:

```cpp
class MaterialManager {
public:
    pxr::UsdShadeMaterial createPBRMaterial(pxr::UsdStageRefPtr stage,
                                          const std::string& material_path,
                                          const MaterialProperties& props) {

        pxr::UsdShadeMaterial material = pxr::UsdShadeMaterial::Define(stage, pxr::SdfPath(material_path));

        // Create surface shader
        std::string shader_path = material_path + "/SurfaceShader";
        pxr::UsdShadeShader shader = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(shader_path));
        shader.CreateIdAttr(pxr::VtValue("UsdPreviewSurface"));

        // Set material properties
        shader.CreateInput("diffuseColor", pxr::SdfValueTypeNames->Color3f)
              .Set(pxr::GfVec3f(props.diffuse_color[0],
                               props.diffuse_color[1],
                               props.diffuse_color[2]));

        shader.CreateInput("metallic", pxr::SdfValueTypeNames->Float)
              .Set(props.metallic);

        shader.CreateInput("roughness", pxr::SdfValueTypeNames->Float)
              .Set(props.roughness);

        shader.CreateInput("specularColor", pxr::SdfValueTypeNames->Color3f)
              .Set(pxr::GfVec3f(props.specular_color[0],
                               props.specular_color[1],
                               props.specular_color[2]));

        shader.CreateInput("opacity", pxr::SdfValueTypeNames->Float)
              .Set(props.opacity);

        if (props.has_emission) {
            shader.CreateInput("emissiveColor", pxr::SdfValueTypeNames->Color3f)
                  .Set(pxr::GfVec3f(props.emission_color[0],
                                   props.emission_color[1],
                                   props.emission_color[2]));
        }

        // Connect shader to material surface
        pxr::UsdShadeOutput surface_output = material.CreateSurfaceOutput();
        pxr::UsdShadeInput surface_input = shader.CreateOutput("surface", pxr::SdfValueTypeNames->Token);
        surface_output.ConnectToSource(shader, "surface");

        return material;
    }

    pxr::UsdShadeMaterial createTexturedMaterial(pxr::UsdStageRefPtr stage,
                                                const std::string& material_path,
                                                const TexturedMaterialProperties& props) {

        pxr::UsdShadeMaterial material = pxr::UsdShadeMaterial::Define(stage, pxr::SdfPath(material_path));

        // Create surface shader
        std::string shader_path = material_path + "/SurfaceShader";
        pxr::UsdShadeShader shader = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(shader_path));
        shader.CreateIdAttr(pxr::VtValue("UsdPreviewSurface"));

        // Create texture samplers
        if (!props.diffuse_texture.empty()) {
            createTextureSampler(stage, material_path + "/DiffuseTexture",
                               props.diffuse_texture, "rgb", shader, "diffuseColor");
        }

        if (!props.metallic_texture.empty()) {
            createTextureSampler(stage, material_path + "/MetallicTexture",
                               props.metallic_texture, "r", shader, "metallic");
        }

        if (!props.roughness_texture.empty()) {
            createTextureSampler(stage, material_path + "/RoughnessTexture",
                               props.roughness_texture, "r", shader, "roughness");
        }

        if (!props.normal_texture.empty()) {
            createNormalMapSampler(stage, material_path + "/NormalTexture",
                                 props.normal_texture, shader);
        }

        // Connect shader to material
        pxr::UsdShadeOutput surface_output = material.CreateSurfaceOutput();
        surface_output.ConnectToSource(shader, "surface");

        return material;
    }

private:
    void createTextureSampler(pxr::UsdStageRefPtr stage,
                             const std::string& sampler_path,
                             const std::string& texture_path,
                             const std::string& channels,
                             pxr::UsdShadeShader& parent_shader,
                             const std::string& input_name) {

        pxr::UsdShadeShader tex_sampler = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(sampler_path));
        tex_sampler.CreateIdAttr(pxr::VtValue("UsdUVTexture"));

        // Set texture file
        tex_sampler.CreateInput("file", pxr::SdfValueTypeNames->Asset)
                  .Set(pxr::SdfAssetPath(texture_path));

        // Set ST coordinate input (use default if not provided)
        pxr::UsdShadeInput st_input = tex_sampler.CreateInput("st", pxr::SdfValueTypeNames->Float2);
        st_input.Set(pxr::GfVec2f(0, 0));  // This would typically connect to UV coordinates

        // Set output channels
        pxr::UsdShadeOutput tex_output = tex_sampler.CreateOutput("rgb", pxr::SdfValueTypeNames->Float3);

        // Connect to parent shader input
        pxr::UsdShadeInput parent_input = parent_shader.CreateInput(input_name, pxr::SdfValueTypeNames->Color3f);
        parent_input.ConnectToSource(tex_sampler, channels);
    }

    void createNormalMapSampler(pxr::UsdStageRefPtr stage,
                               const std::string& sampler_path,
                               const std::string& texture_path,
                               pxr::UsdShadeShader& parent_shader) {

        pxr::UsdShadeShader normal_tex = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(sampler_path));
        normal_tex.CreateIdAttr(pxr::VtValue("UsdUVTexture"));

        normal_tex.CreateInput("file", pxr::SdfValueTypeNames->Asset)
                 .Set(pxr::SdfAssetPath(texture_path));

        // Create normal map conversion shader
        std::string normal_converter_path = sampler_path + "_Converter";
        pxr::UsdShadeShader normal_converter = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(normal_converter_path));
        normal_converter.CreateIdAttr(pxr::VtValue("UsdPreviewSurface"));

        pxr::UsdShadeOutput normal_output = normal_converter.CreateOutput("normal", pxr::SdfValueTypeNames->Normal3f);

        // Connect to shader's normal input
        pxr::UsdShadeInput normal_input = parent_shader.CreateInput("normal", pxr::SdfValueTypeNames->Normal3f);
        normal_input.ConnectToSource(normal_output);
    }

    struct MaterialProperties {
        std::array<float, 3> diffuse_color = {0.8f, 0.8f, 0.8f};
        float metallic = 0.0f;
        float roughness = 0.4f;
        std::array<float, 3> specular_color = {1.0f, 1.0f, 1.0f};
        float opacity = 1.0f;
        bool has_emission = false;
        std::array<float, 3> emission_color = {0.0f, 0.0f, 0.0f};
    };

    struct TexturedMaterialProperties {
        std::string diffuse_texture;
        std::string metallic_texture;
        std::string roughness_texture;
        std::string normal_texture;
        std::string ao_texture;
        MaterialProperties base_properties;
    };
};
```

## Advanced USD Features

### Variants and Layering

USD variants enable multiple configurations of the same asset:

```cpp
class USDVariantManager {
public:
    pxr::UsdVariantSet createConfigurationVariantSet(pxr::UsdPrim& prim,
                                                    const std::string& variant_set_name) {

        pxr::UsdVariantSet variant_set = prim.GetVariantSet(variant_set_name);

        return variant_set;
    }

    void addRobotConfigurationVariant(pxr::UsdPrim& robot_prim,
                                     const std::string& config_name,
                                     const RobotConfiguration& config) {

        pxr::UsdVariantSet config_set = robot_prim.GetVariantSet("Configurations");
        config_set.AddVariant(config_name);
        config_set.SetVariantSelection(config_name);

        // Enter variant edit context and apply configuration
        pxr::UsdEditTarget variant_edit_target = config_set.GetVariantEditTarget();
        pxr::UsdStageRefPtr stage = robot_prim.GetStage();
        stage->SetEditTarget(variant_edit_target);

        // Apply configuration changes
        applyConfigurationToRobot(robot_prim, config);

        // Reset to default edit target
        stage->SetEditTarget(stage->GetRootLayer());
    }

    void addPayloadVariant(pxr::UsdPrim& robot_prim,
                          const std::string& payload_name,
                          const PayloadConfiguration& payload) {

        pxr::UsdVariantSet payload_set = robot_prim.GetVariantSet("Payloads");
        payload_set.AddVariant(payload_name);
        payload_set.SetVariantSelection(payload_name);

        // Apply payload configuration in variant context
        pxr::UsdEditTarget variant_edit_target = payload_set.GetVariantEditTarget();
        pxr::UsdStageRefPtr stage = robot_prim.GetStage();
        stage->SetEditTarget(variant_edit_target);

        applyPayloadToRobot(robot_prim, payload);

        // Reset edit target
        stage->SetEditTarget(stage->GetRootLayer());
    }

private:
    void applyConfigurationToRobot(const pxr::UsdPrim& robot_prim,
                                  const RobotConfiguration& config) {

        // Apply joint limit changes
        for (const auto& joint_limit : config.joint_limits) {
            pxr::UsdPrim joint_prim = robot_prim.GetChild(pxr::TfToken(joint_limit.joint_name));
            if (joint_prim) {
                // Update joint limits
                // This would involve modifying PhysicsJointAPI attributes
            }
        }

        // Apply link property changes
        for (const auto& link_prop : config.link_properties) {
            pxr::UsdPrim link_prim = robot_prim.GetChild(pxr::TfToken(link_prop.link_name));
            if (link_prim) {
                // Update mass, inertia, etc.
                pxr::UsdPhysicsMassAPI mass_api = pxr::UsdPhysicsMassAPI::Apply(link_prim);
                if (mass_api) {
                    mass_api.GetMassAttr().Set(link_prop.mass);
                    mass_api.GetDiagonalInertiaAttr().Set(link_prop.inertia);
                }
            }
        }
    }

    void applyPayloadToRobot(const pxr::UsdPrim& robot_prim,
                            const PayloadConfiguration& payload) {

        // Add payload as child prim
        std::string payload_path = robot_prim.GetPath().GetString() + "/payload_attachment";
        pxr::UsdGeomXform payload_prim = pxr::UsdGeomXform::Define(
            robot_prim.GetStage(), pxr::SdfPath(payload_path));

        // Create payload geometry
        pxr::UsdGeomMesh payload_mesh = pxr::UsdGeomMesh::Define(
            robot_prim.GetStage(),
            pxr::SdfPath(payload_path + "/mesh"));

        // Apply payload mass properties
        pxr::UsdPhysicsMassAPI mass_api = pxr::UsdPhysicsMassAPI::Apply(payload_prim);
        mass_api.GetMassAttr().Set(payload.mass);

        // Apply transform offset
        payload_prim.AddTranslateOp().Set(payload.offset);
    }

    struct RobotConfiguration {
        std::vector<JointLimit> joint_limits;
        std::vector<LinkProperty> link_properties;
        std::string name;
    };

    struct JointLimit {
        std::string joint_name;
        float lower_limit;
        float upper_limit;
        float max_velocity;
        float max_effort;
    };

    struct LinkProperty {
        std::string link_name;
        float mass;
        pxr::GfVec3f inertia;  // Ixx, Iyy, Izz
    };

    struct PayloadConfiguration {
        std::string name;
        float mass;
        pxr::GfVec3f offset;  // Position offset from attachment point
        std::string geometry_path;  // Path to payload geometry
    };
};
```

### Asset Composition and Referencing

USD's powerful composition system allows for modular asset creation:

```cpp
class USDAssetComposer {
public:
    void createModularRobot(pxr::UsdStageRefPtr stage,
                           const std::string& robot_name,
                           const std::vector<std::string>& component_paths) {

        pxr::SdfPath robot_path("/" + robot_name);
        pxr::UsdGeomXform robot_root = pxr::UsdGeomXform::Define(stage, robot_path);

        // Add references to each component
        for (const auto& component_path : component_paths) {
            addComponentReference(stage, robot_path, component_path);
        }
    }

    void createCompositeAsset(const std::string& output_path,
                             const std::vector<std::string>& input_paths) {

        pxr::UsdStageRefPtr stage = pxr::UsdStage::CreateNew(output_path);

        for (size_t i = 0; i < input_paths.size(); ++i) {
            std::string asset_name = "component_" + std::to_string(i);
            pxr::SdfPath asset_path("/" + asset_name);

            // Add reference to component asset
            pxr::UsdGeomXform component = pxr::UsdGeomXform::Define(stage, asset_path);
            component.GetPrim().GetReferences().AddReference(input_paths[i]);
        }

        stage->GetRootLayer()->Save();
    }

    void createPayloadLibrary(pxr::UsdStageRefPtr stage) {
        // Create a library of common payloads that can be referenced
        createPayloadAsset(stage, "/PayloadLibrary/HeavyPayload", 10.0f, "box");
        createPayloadAsset(stage, "/PayloadLibrary/LightPayload", 1.0f, "cylinder");
        createPayloadAsset(stage, "/PayloadLibrary/MediumPayload", 5.0f, "sphere");
    }

private:
    void addComponentReference(pxr::UsdStageRefPtr stage,
                              const pxr::SdfPath& parent_path,
                              const std::string& component_path) {

        std::string component_name = extractComponentName(component_path);
        pxr::SdfPath component_full_path = parent_path.AppendChild(pxr::TfToken(component_name));

        pxr::UsdGeomXform component = pxr::UsdGeomXform::Define(stage, component_full_path);
        component.GetPrim().GetReferences().AddReference(component_path);

        // Apply any component-specific transforms or adjustments
        applyComponentTransforms(component, component_name);
    }

    void applyComponentTransforms(pxr::UsdGeomXform& component, const std::string& component_name) {
        // Apply default transforms based on component type
        if (component_name.find("base") != std::string::npos) {
            component.AddTranslateOp().Set(pxr::GfVec3f(0, 0, 0));
        } else if (component_name.find("arm") != std::string::npos) {
            component.AddTranslateOp().Set(pxr::GfVec3f(0, 0, 0.1));
        } else if (component_name.find("end_effector") != std::string::npos) {
            component.AddTranslateOp().Set(pxr::GfVec3f(0, 0, 0.5));
        }
    }

    std::string extractComponentName(const std::string& path) {
        // Extract component name from path
        size_t last_slash = path.find_last_of("/");
        size_t last_dot = path.find_last_of(".");

        if (last_slash != std::string::npos && last_dot != std::string::npos) {
            return path.substr(last_slash + 1, last_dot - last_slash - 1);
        }

        return "component";
    }

    void createPayloadAsset(pxr::UsdStageRefPtr stage,
                           const std::string& payload_path,
                           float mass,
                           const std::string& geometry_type) {

        pxr::UsdGeomXform payload = pxr::UsdGeomXform::Define(stage, pxr::SdfPath(payload_path));

        // Create appropriate geometry based on type
        pxr::SdfPath geometry_path = pxr::SdfPath(payload_path + "/geometry");

        if (geometry_type == "box") {
            pxr::UsdGeomCube cube = pxr::UsdGeomCube::Define(stage, geometry_path);
            cube.GetSizeAttr().Set(0.1f);
        } else if (geometry_type == "cylinder") {
            pxr::UsdGeomCylinder cyl = pxr::UsdGeomCylinder::Define(stage, geometry_path);
            cyl.GetRadiusAttr().Set(0.05f);
            cyl.GetHeightAttr().Set(0.1f);
        } else if (geometry_type == "sphere") {
            pxr::UsdGeomSphere sphere = pxr::UsdGeomSphere::Define(stage, geometry_path);
            sphere.GetRadiusAttr().Set(0.05f);
        }

        // Add mass properties
        pxr::UsdPhysicsMassAPI mass_api = pxr::UsdPhysicsMassAPI::Apply(payload);
        mass_api.GetMassAttr().Set(mass);
    }
};
```

## Physics Integration

### Collision and Mass Properties

Integrating physics properties with USD assets:

```cpp
class PhysicsAssetIntegrator {
public:
    void addPhysicsPropertiesToRobot(pxr::UsdStageRefPtr stage,
                                    const std::string& robot_path,
                                    const PhysicsConfiguration& config) {

        pxr::UsdPrim robot_prim = stage->GetPrimAtPath(pxr::SdfPath(robot_path));
        if (!robot_prim) return;

        // Process each link in the robot
        for (const auto& child : robot_prim.GetChildren()) {
            if (isLink(child)) {
                addPhysicsPropertiesToLink(stage, child.GetPath().GetString(), config);
            }
        }
    }

    void createArticulation(pxr::UsdStageRefPtr stage,
                           const std::string& robot_path,
                           const std::vector<JointDefinition>& joints) {

        // Apply articulation API to root
        pxr::UsdPhysicsArticulationRootAPI art_root_api =
            pxr::UsdPhysicsArticulationRootAPI::Apply(
                stage->GetPrimAtPath(pxr::SdfPath(robot_path)));

        art_root_api.GetEnabledSelfCollisionsAttr().Set(false);

        // Create joints between links
        for (const auto& joint_def : joints) {
            createJoint(stage, robot_path, joint_def);
        }
    }

private:
    void addPhysicsPropertiesToLink(pxr::UsdStageRefPtr stage,
                                   const std::string& link_path,
                                   const PhysicsConfiguration& config) {

        pxr::UsdPrim link_prim = stage->GetPrimAtPath(pxr::SdfPath(link_path));

        // Apply rigid body API
        pxr::UsdPhysicsRigidBodyAPI rigid_body_api =
            pxr::UsdPhysicsRigidBodyAPI::Apply(link_prim);

        if (rigid_body_api) {
            // Set rigid body properties
            rigid_body_api.GetRigidBodyEnabledAttr().Set(true);
        }

        // Apply mass properties
        pxr::UsdPhysicsMassAPI mass_api = pxr::UsdPhysicsMassAPI::Apply(link_prim);
        if (mass_api) {
            auto mass_props = config.getLinkMassProperties(extractLinkName(link_path));
            mass_api.GetMassAttr().Set(mass_props.mass);
            mass_api.GetCenterOfMassAttr().Set(mass_props.center_of_mass);
            mass_api.GetDiagonalInertiaAttr().Set(mass_props.inertia);
        }

        // Apply collision properties
        for (const auto& child : link_prim.GetChildren()) {
            if (isCollisionGeometry(child)) {
                pxr::UsdPhysicsCollisionAPI collision_api =
                    pxr::UsdPhysicsCollisionAPI::Apply(child);

                if (collision_api) {
                    collision_api.GetCollisionEnabledAttr().Set(true);

                    // Set collision approximation
                    collision_api.GetApproximationAttr().Set(
                        pxr::TfToken("convexHull")); // or "mesh", "boundingBox", etc.
                }
            }
        }
    }

    void createJoint(pxr::UsdStageRefPtr stage,
                    const std::string& robot_path,
                    const JointDefinition& joint_def) {

        std::string joint_path = robot_path + "/" + joint_def.name;

        pxr::UsdPrim joint_prim;
        switch (joint_def.type) {
            case JointType::REVOLUTE:
                joint_prim = pxr::UsdPhysicsRevoluteJoint::Define(stage, pxr::SdfPath(joint_path)).GetPrim();
                break;
            case JointType::PRISMATIC:
                joint_prim = pxr::UsdPhysicsPrismaticJoint::Define(stage, pxr::SdfPath(joint_path)).GetPrim();
                break;
            case JointType::FIXED:
                joint_prim = pxr::UsdPhysicsFixedJoint::Define(stage, pxr::SdfPath(joint_path)).GetPrim();
                break;
            case JointType::SPHERICAL:
                joint_prim = pxr::UsdPhysicsSphericalJoint::Define(stage, pxr::SdfPath(joint_path)).GetPrim();
                break;
            default:
                return; // Unsupported joint type
        }

        if (joint_prim) {
            // Set joint properties
            pxr::UsdPhysicsJoint joint_schema(joint_prim);

            // Set body connections
            joint_schema.GetBody0Rel().AddTarget(pxr::SdfPath(joint_def.parent_link));
            joint_schema.GetBody1Rel().AddTarget(pxr::SdfPath(joint_def.child_link));

            // Set joint transforms
            joint_schema.GetLocalPos0Attr().Set(joint_def.parent_frame.translation);
            joint_schema.GetLocalRot0Attr().Set(joint_def.parent_frame.rotation);
            joint_schema.GetLocalPos1Attr().Set(joint_def.child_frame.translation);
            joint_schema.GetLocalRot1Attr().Set(joint_def.child_frame.rotation);

            // Set joint limits if applicable
            if (joint_def.has_limits) {
                if (auto revolute_joint = pxr::UsdPhysicsRevoluteJoint(joint_prim)) {
                    revolute_joint.GetLowerLimitAttr().Set(joint_def.lower_limit);
                    revolute_joint.GetUpperLimitAttr().Set(joint_def.upper_limit);
                }
            }

            // Set drive properties
            if (joint_def.has_drive) {
                configureJointDrive(joint_prim, joint_def.drive);
            }
        }
    }

    void configureJointDrive(const pxr::UsdPrim& joint_prim, const JointDrive& drive) {
        // Configure joint drives for actuation
        if (auto revolute_joint = pxr::UsdPhysicsRevoluteJoint(joint_prim)) {
            if (drive.enable_position_drive) {
                revolute_joint.GetEnablePositionLimitAttr().Set(true);
                revolute_joint.GetStiffnessAttr().Set(drive.position_stiffness);
                revolute_joint.GetDampingAttr().Set(drive.position_damping);
            }

            if (drive.enable_velocity_drive) {
                revolute_joint.GetEnableVelocityLimitAttr().Set(true);
                revolute_joint.GetMaxVelocityAttr().Set(drive.max_velocity);
            }
        }
    }

    bool isLink(const pxr::UsdPrim& prim) {
        // Check if prim represents a robot link
        std::string prim_name = prim.GetName().GetString();
        return (prim_name.find("link") != std::string::npos) ||
               (prim_name.find("base") != std::string::npos) ||
               (prim_name.find("arm") != std::string::npos) ||
               (prim_name.find("hand") != std::string::npos);
    }

    bool isCollisionGeometry(const pxr::UsdPrim& prim) {
        // Check if prim represents collision geometry
        std::string prim_name = prim.GetName().GetString();
        return (prim_name.find("collision") != std::string::npos) ||
               (prim_name.find("collider") != std::string::npos);
    }

    std::string extractLinkName(const std::string& path) {
        // Extract link name from path
        size_t last_slash = path.find_last_of("/");
        if (last_slash != std::string::npos) {
            return path.substr(last_slash + 1);
        }
        return path;
    }

    enum class JointType { REVOLUTE, PRISMATIC, FIXED, SPHERICAL };

    struct JointDefinition {
        std::string name;
        JointType type;
        std::string parent_link;
        std::string child_link;
        Transform parent_frame;
        Transform child_frame;
        bool has_limits = false;
        float lower_limit = 0.0f;
        float upper_limit = 0.0f;
        bool has_drive = false;
        JointDrive drive;
    };

    struct Transform {
        pxr::GfVec3f translation;
        pxr::GfQuatf rotation;
    };

    struct JointDrive {
        bool enable_position_drive = false;
        bool enable_velocity_drive = false;
        float position_stiffness = 1e7f;
        float position_damping = 1e5f;
        float max_velocity = 100.0f;
    };

    struct LinkMassProperties {
        float mass;
        pxr::GfVec3f center_of_mass;
        pxr::GfVec3f inertia;  // Ixx, Iyy, Izz
    };

    struct PhysicsConfiguration {
        std::map<std::string, LinkMassProperties> link_mass_properties;

        LinkMassProperties getLinkMassProperties(const std::string& link_name) {
            auto it = link_mass_properties.find(link_name);
            if (it != link_mass_properties.end()) {
                return it->second;
            }
            // Return default properties if not found
            return {1.0f, pxr::GfVec3f(0, 0, 0), pxr::GfVec3f(0.1f, 0.1f, 0.1f)};
        }
    };
};
```

## Asset Optimization Techniques

### Level of Detail (LOD) Management

Creating multiple detail levels for efficient rendering:

```cpp
class USDLODManager {
public:
    void createLODLevels(pxr::UsdStageRefPtr stage,
                        const std::string& base_path,
                        const std::vector<LODLevel>& lod_levels) {

        pxr::UsdGeomLOD lod_prim = pxr::UsdGeomLOD::Define(stage, pxr::SdfPath(base_path + "/LOD"));

        // Set range scales for each LOD
        VtFloatArray ranges;
        VtTokenArray variants;

        for (const auto& lod_level : lod_levels) {
            ranges.push_back(lod_level.switch_range);
            variants.push_back(pxr::TfToken(lod_level.level_name));

            // Create variant for this LOD level
            createLODVariant(stage, base_path, lod_level);
        }

        lod_prim.GetUsdGeomLOD().GetVariantSelection()
              .Set(pxr::TfToken(lod_levels[0].level_name)); // Default to highest detail
    }

    void optimizeForSimulation(pxr::UsdStageRefPtr stage,
                              const std::string& asset_path,
                              bool for_physics = true,
                              bool for_rendering = true) {

        pxr::UsdPrim asset_prim = stage->GetPrimAtPath(pxr::SdfPath(asset_path));
        if (!asset_prim) return;

        for (const auto& child : asset_prim.GetChildren()) {
            optimizePrimitive(child, for_physics, for_rendering);
        }
    }

private:
    void createLODVariant(pxr::UsdStageRefPtr stage,
                         const std::string& base_path,
                         const LODLevel& lod_level) {

        std::string variant_path = base_path + "/" + lod_level.level_name;
        pxr::UsdGeomXform lod_variant = pxr::UsdGeomXform::Define(stage, pxr::SdfPath(variant_path));

        // Apply the appropriate mesh or simplified representation
        if (lod_level.geometry_path.empty()) {
            // Create simplified geometry procedurally
            createSimplifiedGeometry(stage, variant_path, lod_level.complexity_factor);
        } else {
            // Reference external simplified geometry
            lod_variant.GetPrim().GetReferences().AddReference(lod_level.geometry_path);
        }
    }

    void createSimplifiedGeometry(pxr::UsdStageRefPtr stage,
                                 const std::string& path,
                                 float complexity_factor) {

        if (complexity_factor < 0.1f) {
            // Use bounding box approximation for very low detail
            pxr::UsdGeomCube bounding_box = pxr::UsdGeomCube::Define(stage, pxr::SdfPath(path + "/bbox"));
            bounding_box.GetSizeAttr().Set(1.0f * sqrt(complexity_factor));
        } else if (complexity_factor < 0.5f) {
            // Use simplified mesh (convex hull or primitive approximation)
            pxr::UsdGeomSphere simplified_sphere = pxr::UsdGeomSphere::Define(
                stage, pxr::SdfPath(path + "/approximation"));
            simplified_sphere.GetRadiusAttr().Set(0.5f);
        } else {
            // Use detailed mesh with reduced polygon count
            // This would involve referencing a pre-processed simplified mesh
        }
    }

    void optimizePrimitive(pxr::UsdPrim& prim,
                          bool for_physics,
                          bool for_rendering) {

        if (for_physics) {
            optimizeForPhysics(prim);
        }

        if (for_rendering) {
            optimizeForRendering(prim);
        }
    }

    void optimizeForPhysics(pxr::UsdPrim& prim) {
        if (prim.IsA<pxr::UsdGeomMesh>()) {
            // For physics, use simplified collision geometry
            pxr::UsdPhysicsCollisionAPI collision_api =
                pxr::UsdPhysicsCollisionAPI::Apply(prim);

            if (collision_api) {
                // Use convex hull approximation for better performance
                collision_api.GetApproximationAttr().Set(pxr::TfToken("convexHull"));
            }
        }
    }

    void optimizeForRendering(pxr::UsdPrim& prim) {
        if (prim.IsA<pxr::UsdGeomMesh>()) {
            auto mesh = pxr::UsdGeomMesh(prim);

            // Set render properties for optimization
            mesh.GetDoubleSidedAttr().Set(true);

            // Consider adding render-level-of-detail properties
            // This would depend on the specific renderer being used
        }
    }

    struct LODLevel {
        std::string level_name;      // "high", "medium", "low", etc.
        float switch_range;          // Distance at which to switch
        std::string geometry_path;   // Path to alternative geometry
        float complexity_factor;     // Factor to determine simplification level
    };
};
```

### Asset Streaming and Loading

Efficient asset loading and streaming:

```cpp
class USDAssetStreamer {
private:
    std::map<std::string, pxr::UsdStageRefPtr> loaded_stages_;
    std::queue<std::string> loading_queue_;
    std::mutex loading_mutex_;
    std::condition_variable loading_cv_;
    bool shutdown_flag_ = false;

public:
    pxr::UsdStageRefPtr loadAssetAsync(const std::string& asset_path) {
        // Check if already loaded
        auto it = loaded_stages_.find(asset_path);
        if (it != loaded_stages_.end()) {
            return it->second;
        }

        // Start async loading
        std::future<pxr::UsdStageRefPtr> future = std::async(
            std::launch::async,
            [asset_path]() -> pxr::UsdStageRefPtr {
                return pxr::UsdStage::Open(asset_path);
            }
        );

        // Wait for loading to complete (in practice, you'd handle this asynchronously)
        pxr::UsdStageRefPtr stage = future.get();

        if (stage) {
            loaded_stages_[asset_path] = stage;
        }

        return stage;
    }

    void preloadAssets(const std::vector<std::string>& asset_paths,
                      size_t max_concurrent_loads = 4) {

        std::vector<std::future<void>> loading_futures;

        for (const auto& path : asset_paths) {
            if (loading_futures.size() < max_concurrent_loads) {
                loading_futures.push_back(
                    std::async(std::launch::async, [this, path]() {
                        loadAssetAsync(path);
                    })
                );
            } else {
                // Wait for one to complete before starting more
                for (auto& future : loading_futures) {
                    if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                        future.get();
                        break;
                    }
                }

                // Start new loading task
                loading_futures.push_back(
                    std::async(std::launch::async, [this, path]() {
                        loadAssetAsync(path);
                    })
                );
            }
        }

        // Wait for all to complete
        for (auto& future : loading_futures) {
            future.get();
        }
    }

    void unloadUnusedAssets(const std::vector<std::string>& active_assets) {
        std::set<std::string> active_set(active_assets.begin(), active_assets.end());

        for (auto it = loaded_stages_.begin(); it != loaded_stages_.end();) {
            if (active_set.find(it->first) == active_set.end()) {
                it = loaded_stages_.erase(it);
            } else {
                ++it;
            }
        }
    }

    pxr::UsdStageRefPtr getLoadedAsset(const std::string& asset_path) {
        auto it = loaded_stages_.find(asset_path);
        if (it != loaded_stages_.end()) {
            return it->second;
        }
        return nullptr;
    }

    void clearCache() {
        loaded_stages_.clear();
    }
};
```

## Integration with Robotics Frameworks

### ROS 2 Integration

Integrating USD assets with ROS 2 robot descriptions:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "pxr/usd/usd/stage.h"

class USDROS2Integrator {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    std::map<std::string, pxr::UsdStageRefPtr> asset_stages_;
    std::map<std::string, pxr::UsdPrim> robot_prims_;

public:
    USDROS2Integrator(rclcpp::Node::SharedPtr node) : node_(node) {
        joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
    }

    bool loadRobotFromUSD(const std::string& robot_name, const std::string& usd_path) {
        pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(usd_path);
        if (!stage) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load USD file: %s", usd_path.c_str());
            return false;
        }

        // Find robot prim in stage
        pxr::UsdPrim robot_prim = stage->GetPseudoRoot();
        for (const auto& child : stage->GetPseudoRoot().GetChildren()) {
            if (isRobotPrim(child)) {
                robot_prim = child;
                break;
            }
        }

        if (!robot_prim) {
            RCLCPP_ERROR(node_->get_logger(), "No robot prim found in USD file: %s", usd_path.c_str());
            return false;
        }

        asset_stages_[robot_name] = stage;
        robot_prims_[robot_name] = robot_prim;

        // Extract joint information from USD
        extractJointInformation(robot_name, robot_prim);

        RCLCPP_INFO(node_->get_logger(), "Successfully loaded robot '%s' from USD", robot_name.c_str());
        return true;
    }

    void publishJointStates(const std::string& robot_name,
                           const std::vector<double>& joint_positions,
                           const std::vector<double>& joint_velocities = {},
                           const std::vector<double>& joint_efforts = {}) {

        if (robot_prims_.find(robot_name) == robot_prims_.end()) {
            return; // Robot not loaded
        }

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = node_->get_clock()->now();
        msg.header.frame_id = robot_name + "_base_link";

        // Get joint names for this robot
        auto joint_names = getJointNames(robot_name);

        if (joint_positions.size() != joint_names.size()) {
            RCLCPP_ERROR(node_->get_logger(), "Joint position size mismatch for robot %s", robot_name.c_str());
            return;
        }

        msg.name = joint_names;
        msg.position = joint_positions;

        if (joint_velocities.size() == joint_names.size()) {
            msg.velocity = joint_velocities;
        }

        if (joint_efforts.size() == joint_names.size()) {
            msg.effort = joint_efforts;
        }

        joint_state_pub_->publish(msg);
    }

private:
    bool isRobotPrim(const pxr::UsdPrim& prim) {
        // Determine if prim represents a robot by checking for common robot attributes
        return prim.IsA<pxr::UsdGeomXform>() &&
               (prim.GetName().GetString().find("robot") != std::string::npos ||
                prim.GetName().GetString().find("Robot") != std::string::npos);
    }

    void extractJointInformation(const std::string& robot_name, const pxr::UsdPrim& robot_prim) {
        // Extract joint information from USD physics schemas
        std::vector<std::string> joint_names;

        for (const auto& child : robot_prim.GetDescendants()) {
            if (isJoint(child)) {
                joint_names.push_back(child.GetName().GetString());
            }
        }

        // Store joint information for this robot
        robot_joint_names_[robot_name] = joint_names;
    }

    bool isJoint(const pxr::UsdPrim& prim) {
        // Check if prim represents a joint
        return prim.IsA<pxr::UsdPhysicsJoint>() ||
               prim.IsA<pxr::UsdPhysicsRevoluteJoint>() ||
               prim.IsA<pxr::UsdPhysicsPrismaticJoint>() ||
               prim.IsA<pxr::UsdPhysicsFixedJoint>();
    }

    std::vector<std::string> getJointNames(const std::string& robot_name) {
        auto it = robot_joint_names_.find(robot_name);
        if (it != robot_joint_names_.end()) {
            return it->second;
        }
        return {};
    }

    std::map<std::string, std::vector<std::string>> robot_joint_names_;
};
```

### Isaac Sim Integration

Specific integration with NVIDIA Isaac Sim:

```cpp
#include "omni/kit/Kit.h"
#include "omni/kit/App.h"
#include "pxr/usd/usd/stage.h"
#include "omni/isaac/core/utils/nucleus.h"
#include "omni/isaac/core/utils/stage.h"

class IsaacUSDIntegrator {
private:
    std::shared_ptr<omni::kit::IKit> kit_;
    pxr::UsdStageRefPtr current_stage_;

public:
    bool initializeIsaacSim() {
        // Initialize Isaac Sim application
        kit_ = omni::kit::App::Get();
        if (!kit_) {
            return false;
        }

        return true;
    }

    bool loadRobotAsset(const std::string& asset_path, const std::string& prim_path) {
        if (!kit_) {
            if (!initializeIsaacSim()) {
                return false;
            }
        }

        // Open USD stage
        current_stage_ = pxr::UsdStage::Open(asset_path);
        if (!current_stage_) {
            return false;
        }

        // Add robot to Isaac Sim stage
        std::string nucleus_url = "omniverse://localhost/NVIDIA/Assets";
        std::string robot_asset_url = nucleus_url + "/" + asset_path;

        // Create robot prim at specified path
        pxr::UsdGeomXform robot_prim = pxr::UsdGeomXform::Define(
            current_stage_, pxr::SdfPath(prim_path));

        // Add reference to robot asset
        robot_prim.GetPrim().GetReferences().AddReference(robot_asset_url);

        // Apply Isaac Sim specific schemas
        applyIsaacSchemas(robot_prim);

        return true;
    }

    void applyIsaacSchemas(const pxr::UsdGeomXform& robot_prim) {
        // Apply Isaac-specific schemas for robotics functionality
        applyArticulationSchema(robot_prim);
        applyDifferentialDriveSchema(robot_prim);
        applyCameraSchema(robot_prim);
    }

    void applyArticulationSchema(const pxr::UsdGeomXform& robot_prim) {
        // Apply articulation schema for robot joints
        pxr::UsdPhysicsArticulationRootAPI art_root_api =
            pxr::UsdPhysicsArticulationRootAPI::Apply(robot_prim.GetPrim());

        if (art_root_api) {
            art_root_api.GetEnabledSelfCollisionsAttr().Set(false);
        }
    }

    void applyDifferentialDriveSchema(const pxr::UsdGeomXform& robot_prim) {
        // Apply differential drive schema if robot is a mobile base
        // This would involve adding specific drive properties
    }

    void applyCameraSchema(const pxr::UsdGeomXform& robot_prim) {
        // Apply camera schemas to any camera prims in the robot
        for (const auto& child : robot_prim.GetPrim().GetChildren()) {
            if (isCameraPrim(child)) {
                applyCameraProperties(child);
            }
        }
    }

    bool isCameraPrim(const pxr::UsdPrim& prim) {
        return prim.IsA<pxr::UsdGeomCamera>();
    }

    void applyCameraProperties(const pxr::UsdPrim& camera_prim) {
        // Apply Isaac Sim camera properties
        if (auto camera = pxr::UsdGeomCamera(camera_prim)) {
            // Set default camera properties suitable for Isaac Sim
            camera.GetFocalLengthAttr().Set(24.0f);  // 24mm focal length
            camera.GetHorizontalApertureAttr().Set(20.955f);  // APS-C sensor width
            camera.GetVerticalApertureAttr().Set(15.2908f);   // APS-C sensor height
        }
    }

    pxr::UsdStageRefPtr getCurrentStage() {
        return current_stage_;
    }

    void setPhysicsProperties(const std::string& prim_path, const PhysicsProperties& props) {
        pxr::UsdPrim prim = current_stage_->GetPrimAtPath(pxr::SdfPath(prim_path));
        if (!prim) return;

        pxr::UsdPhysicsRigidBodyAPI rigid_body_api =
            pxr::UsdPhysicsRigidBodyAPI::Apply(prim);

        if (rigid_body_api) {
            rigid_body_api.GetLinearDampingAttr().Set(props.linear_damping);
            rigid_body_api.GetAngularDampingAttr().Set(props.angular_damping);
            rigid_body_api.GetMaxLinearVelocityAttr().Set(props.max_linear_velocity);
            rigid_body_api.GetMaxAngularVelocityAttr().Set(props.max_angular_velocity);
        }
    }

    struct PhysicsProperties {
        float linear_damping = 0.05f;
        float angular_damping = 0.01f;
        float max_linear_velocity = 1000.0f;
        float max_angular_velocity = 50.0f;
        bool enable_gravity = true;
        bool visualize = true;
    };
};
```

## Asset Validation and Quality Assurance

### USD Asset Validation

Validating USD assets for simulation readiness:

```cpp
class USDAssetValidator {
public:
    struct ValidationResult {
        bool is_valid = false;
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
        std::vector<std::string> info_messages;
        double quality_score = 0.0;  // 0.0 to 1.0
    };

    ValidationResult validateRobotAsset(const std::string& asset_path) {
        ValidationResult result;

        pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(asset_path);
        if (!stage) {
            result.errors.push_back("Failed to open USD stage: " + asset_path);
            return result;
        }

        // Check stage integrity
        if (!stage->GetPseudoRoot()) {
            result.errors.push_back("USD stage has no valid root prim");
            return result;
        }

        // Validate robot structure
        auto robot_prims = findRobotPrims(stage);
        if (robot_prims.empty()) {
            result.warnings.push_back("No robot prims found in asset");
        }

        // Validate each robot
        for (const auto& robot_prim : robot_prims) {
            auto robot_validation = validateRobotStructure(robot_prim);
            result.errors.insert(result.errors.end(),
                               robot_validation.errors.begin(),
                               robot_validation.errors.end());
            result.warnings.insert(result.warnings.end(),
                                  robot_validation.warnings.begin(),
                                  robot_validation.warnings.end());
        }

        // Validate physics properties
        auto physics_validation = validatePhysicsProperties(stage);
        result.errors.insert(result.errors.end(),
                           physics_validation.errors.begin(),
                           physics_validation.errors.end());
        result.warnings.insert(result.warnings.end(),
                              physics_validation.warnings.begin(),
                              physics_validation.warnings.end());

        // Validate materials and shaders
        auto material_validation = validateMaterials(stage);
        result.errors.insert(result.errors.end(),
                           material_validation.errors.begin(),
                           material_validation.errors.end());
        result.warnings.insert(result.warnings.end(),
                              material_validation.warnings.begin(),
                              material_validation.warnings.end());

        // Calculate quality score
        result.quality_score = calculateQualityScore(result);
        result.is_valid = result.errors.empty();

        return result;
    }

private:
    std::vector<pxr::UsdPrim> findRobotPrims(pxr::UsdStageRefPtr stage) {
        std::vector<pxr::UsdPrim> robot_prims;

        for (const auto& prim : stage->Traverse()) {
            if (isRobotPrim(prim)) {
                robot_prims.push_back(prim);
            }
        }

        return robot_prims;
    }

    bool isRobotPrim(const pxr::UsdPrim& prim) {
        // Check if prim represents a robot structure
        if (!prim.IsA<pxr::UsdGeomXform>()) {
            return false;
        }

        std::string prim_name = prim.GetName().GetString();
        if (prim_name.find("robot") != std::string::npos ||
            prim_name.find("Robot") != std::string::npos ||
            prim_name.find("arm") != std::string::npos ||
            prim_name.find("Arm") != std::string::npos) {
            return true;
        }

        // Check for robot-like structure (has links and joints)
        bool has_links = false;
        bool has_joints = false;

        for (const auto& child : prim.GetChildren()) {
            if (isLinkPrim(child)) has_links = true;
            if (isJointPrim(child)) has_joints = true;
        }

        return has_links && has_joints;
    }

    bool isLinkPrim(const pxr::UsdPrim& prim) {
        std::string prim_name = prim.GetName().GetString();
        return (prim_name.find("link") != std::string::npos) ||
               (prim_name.find("Link") != std::string::npos) ||
               (prim_name.find("body") != std::string::npos) ||
               (prim_name.find("Body") != std::string::npos);
    }

    bool isJointPrim(const pxr::UsdPrim& prim) {
        return prim.IsA<pxr::UsdPhysicsJoint>() ||
               prim.IsA<pxr::UsdPhysicsRevoluteJoint>() ||
               prim.IsA<pxr::UsdPhysicsPrismaticJoint>() ||
               prim.IsA<pxr::UsdPhysicsFixedJoint>();
    }

    ValidationResult validateRobotStructure(const pxr::UsdPrim& robot_prim) {
        ValidationResult result;

        // Check for base link
        bool has_base_link = false;
        for (const auto& child : robot_prim.GetChildren()) {
            if (isBaseLink(child)) {
                has_base_link = true;
                break;
            }
        }

        if (!has_base_link) {
            result.warnings.push_back("Robot has no identifiable base link");
        }

        // Check joint connectivity
        auto joints = getJoints(robot_prim);
        auto links = getLinks(robot_prim);

        for (const auto& joint : joints) {
            auto joint_validation = validateJointConnection(joint, links);
            result.errors.insert(result.errors.end(),
                               joint_validation.errors.begin(),
                               joint_validation.errors.end());
            result.warnings.insert(result.warnings.end(),
                                  joint_validation.warnings.begin(),
                                  joint_validation.warnings.end());
        }

        // Check for complete kinematic chain
        if (!validateKinematicChain(robot_prim)) {
            result.warnings.push_back("Incomplete or malformed kinematic chain detected");
        }

        return result;
    }

    bool isBaseLink(const pxr::UsdPrim& prim) {
        std::string prim_name = prim.GetName().GetString();
        return (prim_name.find("base") != std::string::npos) ||
               (prim_name.find("Base") != std::string::npos) ||
               (prim_name.find("root") != std::string::npos) ||
               (prim_name.find("Root") != std::string::npos);
    }

    std::vector<pxr::UsdPrim> getJoints(const pxr::UsdPrim& robot_prim) {
        std::vector<pxr::UsdPrim> joints;

        for (const auto& child : robot_prim.GetDescendants()) {
            if (isJointPrim(child)) {
                joints.push_back(child);
            }
        }

        return joints;
    }

    std::vector<pxr::UsdPrim> getLinks(const pxr::UsdPrim& robot_prim) {
        std::vector<pxr::UsdPrim> links;

        for (const auto& child : robot_prim.GetDescendants()) {
            if (isLinkPrim(child)) {
                links.push_back(child);
            }
        }

        return links;
    }

    ValidationResult validateJointConnection(const pxr::UsdPrim& joint_prim,
                                           const std::vector<pxr::UsdPrim>& links) {
        ValidationResult result;

        // Check if joint has proper body connections
        pxr::UsdPhysicsJoint joint_schema(joint_prim);
        if (!joint_schema) {
            result.errors.push_back("Invalid joint schema: " + joint_prim.GetPath().GetString());
            return result;
        }

        auto body0_targets = joint_schema.GetBody0Rel().GetTargets();
        auto body1_targets = joint_schema.GetBody1Rel().GetTargets();

        if (body0_targets.empty()) {
            result.errors.push_back("Joint " + joint_prim.GetPath().GetString() + " has no body0 connection");
        }

        if (body1_targets.empty()) {
            result.errors.push_back("Joint " + joint_prim.GetPath().GetString() + " has no body1 connection");
        }

        // Verify that connected bodies exist
        for (const auto& target : body0_targets) {
            if (!std::any_of(links.begin(), links.end(),
                           [&target](const pxr::UsdPrim& link) {
                               return link.GetPath() == target;
                           })) {
                result.errors.push_back("Joint body0 target doesn't exist: " + target.GetString());
            }
        }

        for (const auto& target : body1_targets) {
            if (!std::any_of(links.begin(), links.end(),
                           [&target](const pxr::UsdPrim& link) {
                               return link.GetPath() == target;
                           })) {
                result.errors.push_back("Joint body1 target doesn't exist: " + target.GetString());
            }
        }

        return result;
    }

    bool validateKinematicChain(const pxr::UsdPrim& robot_prim) {
        // Check if joints form a proper kinematic chain
        // This is a simplified check - in practice, this would be more complex
        auto joints = getJoints(robot_prim);
        auto links = getLinks(robot_prim);

        // For a proper kinematic chain, each link (except base) should be connected to exactly one joint
        std::set<std::string> connected_links;
        for (const auto& joint : joints) {
            pxr::UsdPhysicsJoint joint_schema(joint);
            auto body1_targets = joint_schema.GetBody1Rel().GetTargets();
            for (const auto& target : body1_targets) {
                connected_links.insert(target.GetString());
            }
        }

        // Count unconnected links (excluding base)
        int unconnected_count = 0;
        for (const auto& link : links) {
            if (link.GetName().GetString().find("base") == std::string::npos &&
                connected_links.find(link.GetPath().GetString()) == connected_links.end()) {
                unconnected_count++;
            }
        }

        return unconnected_count == 0;  // All non-base links should be connected
    }

    ValidationResult validatePhysicsProperties(pxr::UsdStageRefPtr stage) {
        ValidationResult result;

        for (const auto& prim : stage->Traverse()) {
            if (prim.IsA<pxr::UsdGeomMesh>()) {
                // Check for collision properties
                pxr::UsdPhysicsCollisionAPI collision_api(prim);
                if (!collision_api) {
                    // Check if this is a visual-only mesh (might be acceptable)
                    std::string prim_name = prim.GetName().GetString();
                    if (prim_name.find("visual") != std::string::npos ||
                        prim_name.find("Visual") != std::string::npos) {
                        continue; // Visual-only meshes don't need collision
                    } else {
                        result.warnings.push_back("Mesh has no collision properties: " + prim.GetPath().GetString());
                    }
                }
            }

            if (prim.IsA<pxr::UsdGeomXform>()) {
                // Check for mass properties
                pxr::UsdPhysicsMassAPI mass_api(prim);
                if (!mass_api) {
                    // This might be acceptable for static objects
                    continue;
                }

                // Validate mass properties
                auto mass_attr = mass_api.GetMassAttr();
                if (mass_attr) {
                    float mass_value;
                    if (mass_attr.Get(&mass_value)) {
                        if (mass_value <= 0) {
                            result.errors.push_back("Invalid mass value (<= 0) for: " + prim.GetPath().GetString());
                        }
                    }
                }
            }
        }

        return result;
    }

    ValidationResult validateMaterials(pxr::UsdStageRefPtr stage) {
        ValidationResult result;

        for (const auto& prim : stage->Traverse()) {
            if (prim.IsA<pxr::UsdShadeMaterial>()) {
                pxr::UsdShadeMaterial material(prim);

                // Check for surface output
                auto surface_output = material.GetSurfaceOutput();
                if (!surface_output) {
                    result.errors.push_back("Material has no surface output: " + prim.GetPath().GetString());
                }

                // Check shader properties
                auto shader_prim = material.ComputeSurfaceSource().GetPrim();
                if (shader_prim) {
                    pxr::UsdShadeShader shader(shader_prim);
                    auto shader_id_attr = shader.GetIdAttr();
                    if (shader_id_attr) {
                        std::string shader_id;
                        if (shader_id_attr.Get(&shader_id)) {
                            if (shader_id != "UsdPreviewSurface") {
                                result.info_messages.push_back("Non-standard shader used: " + shader_id);
                            }
                        }
                    }
                }
            }
        }

        return result;
    }

    double calculateQualityScore(const ValidationResult& result) {
        // Calculate quality score based on validation results
        int total_issues = result.errors.size() + result.warnings.size();
        int error_penalty = result.errors.size() * 10;  // Errors heavily penalize score
        int warning_penalty = result.warnings.size();   // Warnings have smaller penalty

        // Base score calculation (inverted penalties)
        double score = 1.0 - (error_penalty + warning_penalty * 0.1) / 100.0;
        return std::max(0.0, std::min(1.0, score));  // Clamp to [0, 1]
    }
};
```

## Best Practices for USD Asset Creation

### Asset Organization

**Hierarchical Structure:**
- Organize assets in logical hierarchies
- Use consistent naming conventions
- Group related components together
- Maintain clear parent-child relationships

**Modular Design:**
- Create reusable components
- Use references for common elements
- Implement variant sets for different configurations
- Design assets for easy modification and extension

### Quality Assurance

**Validation Procedures:**
- Implement automated validation checks
- Test assets in multiple simulation environments
- Verify physics properties and constraints
- Check for geometric and topological errors

**Performance Optimization:**
- Use appropriate level of detail for different use cases
- Optimize mesh complexity where possible
- Implement efficient material definitions
- Consider streaming and caching strategies

### Documentation and Maintenance

**Asset Metadata:**
- Include version information
- Document author and creation date
- Specify intended use cases and limitations
- Provide performance characteristics

**Change Management:**
- Maintain version control for assets
- Document changes and improvements
- Establish approval processes for asset updates
- Track asset usage and dependencies

## Advanced USD Features

### Custom Schemas

Creating domain-specific USD schemas for robotics:

```cpp
// Custom schema example for robot-specific properties
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

PXR_NAMESPACE_OPEN_SCOPE

class RobotComponentAPI : public pxr::UsdAPISchemaBase
{
public:
    static RobotComponentAPI Apply(const pxr::UsdPrim& prim, const TfToken &instance = TfToken());

    pxr::UsdAttribute GetRobotComponentTypeAttr() const;
    pxr::UsdAttribute GetMaxOperatingTempAttr() const;
    pxr::UsdAttribute GetPowerConsumptionAttr() const;

private:
    explicit RobotComponentAPI(const pxr::UsdPrim& prim);
};

PXR_NAMESPACE_CLOSE_SCOPE
```

### MaterialX Integration

Using MaterialX for advanced material definitions:

```cpp
#include "MaterialXCore/Document.h"
#include "MaterialXFormat/XmlIo.h"

class MaterialXUSDConverter {
public:
    pxr::UsdShadeMaterial convertMaterialXToUSD(
        const std::string& materialx_path,
        pxr::UsdStageRefPtr stage,
        const std::string& material_path) {

        // Load MaterialX document
        mx::DocumentPtr doc = mx::createDocument();
        mx::readFromXmlFile(doc, materialx_path);

        // Convert to USD material
        pxr::UsdShadeMaterial usd_material = pxr::UsdShadeMaterial::Define(
            stage, pxr::SdfPath(material_path));

        // Convert MaterialX nodes to USD shaders
        for (mx::ElementPtr elem : doc->getChildren()) {
            if (elem->isA<mx::Node>()) {
                auto node = elem->asA<mx::Node>();
                convertNodeToUSDShader(node, usd_material, stage);
            }
        }

        return usd_material;
    }

private:
    void convertNodeToUSDShader(const mx::NodePtr& mx_node,
                               pxr::UsdShadeMaterial& usd_material,
                               pxr::UsdStageRefPtr stage) {
        // Convert MaterialX node to USD shader
        std::string shader_type = mx_node->getCategory();
        std::string node_name = mx_node->getName();

        std::string shader_path = usd_material.GetPath().GetString() + "/" + node_name;
        pxr::UsdShadeShader shader = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(shader_path));

        // Map MaterialX shader type to USD equivalent
        std::string usd_shader_id = mapShaderType(shader_type);
        shader.CreateIdAttr(pxr::VtValue(usd_shader_id));

        // Convert MaterialX parameters to USD inputs
        for (mx::ParameterPtr param : mx_node->getParameters()) {
            convertParameterToUSDInput(param, shader);
        }

        // Connect shader to material output
        pxr::UsdShadeOutput surface_output = usd_material.CreateSurfaceOutput();
        surface_output.ConnectToSource(shader, "surface");
    }

    std::string mapShaderType(const std::string& mx_type) {
        // Map MaterialX shader types to USD equivalents
        if (mx_type == "surface") {
            return "UsdPreviewSurface";
        } else if (mx_type == "pbr") {
            return "UsdPreviewSurface";
        } else {
            return "UsdPreviewSurface";  // Default fallback
        }
    }

    void convertParameterToUSDInput(const mx::ParameterPtr& mx_param,
                                   pxr::UsdShadeShader& shader) {
        std::string param_name = mx_param->getName();
        std::string param_value = mx_param->getValueString();

        // Convert based on parameter type
        if (param_name == "diffuse" || param_name == "base_color") {
            pxr::GfVec3f color_value = parseColorValue(param_value);
            shader.CreateInput("diffuseColor", pxr::SdfValueTypeNames->Color3f).Set(color_value);
        } else if (param_name == "metallic") {
            float float_value = std::stof(param_value);
            shader.CreateInput("metallic", pxr::SdfValueTypeNames->Float).Set(float_value);
        } else if (param_name == "roughness") {
            float float_value = std::stof(param_value);
            shader.CreateInput("roughness", pxr::SdfValueTypeNames->Float).Set(float_value);
        }
    }

    pxr::GfVec3f parseColorValue(const std::string& color_str) {
        // Parse color value from string (e.g., "0.8 0.8 0.8")
        std::istringstream iss(color_str);
        float r, g, b;
        iss >> r >> g >> b;
        return pxr::GfVec3f(r, g, b);
    }
};
```

## Troubleshooting Common Issues

### USD Loading Problems

**Common Issues and Solutions:**

1. **Asset Path Resolution:**
   - Verify asset paths are accessible
   - Check for relative vs. absolute path issues
   - Ensure proper file permissions

2. **Schema Validation Failures:**
   - Check for missing USD plugin dependencies
   - Verify schema compatibility
   - Update schemas to match USD version

3. **Performance Issues:**
   - Reduce mesh complexity where possible
   - Use appropriate level of detail
   - Optimize material definitions

### Physics Simulation Problems

**Collision Detection Issues:**
- Verify collision geometry is properly defined
- Check for missing collision APIs
- Ensure proper scaling of collision meshes

**Dynamic Behavior Problems:**
- Validate mass properties are realistic
- Check for proper inertia tensors
- Verify joint limits and constraints

## Future Developments

### AI-Enhanced Asset Creation

**Procedural Generation:**
- Neural networks for automatic asset creation
- Generative models for environment generation
- AI-assisted material definition

**Style Transfer:**
- Applying visual styles to USD assets
- Automatic texture generation
- Material property optimization

### Advanced Rendering Integration

**Neural Rendering:**
- Integrating neural radiance fields with USD
- AI-enhanced lighting simulation
- Learned appearance models

**Real-time Ray Tracing:**
- Enhanced RTX rendering capabilities
- Advanced global illumination
- Physically-based material simulation

## Conclusion

USD assets represent a powerful and flexible framework for creating realistic simulation environments in robotics. The combination of hierarchical scene description, powerful composition capabilities, and extensible schema system makes USD ideal for representing complex robotic systems with their associated physics, materials, and behaviors. Successful implementation of USD assets requires careful attention to geometric accuracy, physical properties, and the integration of various simulation elements. The modular nature of USD enables the creation of reusable, configurable assets that can be adapted for various applications and environments. As robotics applications become more sophisticated and demand higher fidelity simulation, USD continues to evolve with advanced features for material definition, physics integration, and performance optimization. The investment in properly structured and validated USD assets significantly improves the realism and utility of robotic simulation environments, ultimately leading to more robust and reliable real-world robot deployments.
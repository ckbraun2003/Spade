// component.hpp -- component type registry (24th spec SL4).
//
// A component's type id is its SERIALIZATION KEY. That is the entire reason it
// is assigned here, by hand, monotonically from zero, instead of derived:
// typeid().hash_code() is not stable across compilers, is not stable across
// builds under some ABIs, and carries no ordering. A saved graph stores these
// numbers, so all three properties are load-bearing.
//
// APPENDING a type is legal: add an enumerator at the end, add a row at the end of
// kComponentTypes below (name, availability, reason), bump kComponentTypeCount. RENUMBERING an
// existing one is not -- it silently reinterprets every scene ever saved. The
// ordering below is frozen, and test_objects_component.cpp pins it.
//
// SL3 applies to every struct here: a component holds CONFIGURATION plus
// REFERENCES (slot indices) into state that already exists elsewhere. None of
// them owns simulation data. That is what keeps the object graph out of the
// snapshot and leaves every parity band byte-unchanged.

#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include <glm/glm.hpp>

#include "objects/object.hpp"

namespace spade::objects {

enum class ComponentTypeId : uint32_t {
    transform = 0,
    body = 1,
    mesh = 2,
    material = 3,
    collider = 4,
    sensor = 5,
    force_element = 6,
    camera = 7,
    behavior = 8,
    fluid = 9,
};

inline constexpr uint32_t kComponentTypeCount = 10u;

// ---------------------------------------------------------------------------
// The ten component types.
// ---------------------------------------------------------------------------

// Placement lives on Object itself; presence of this component is the marker
// that the object participates in transform composition at all.
struct TransformComponent {};

struct BodyComponent {
    uint32_t world_index = 0;
    uint32_t body_slot = 0;  // index into the bodies arena; NOT owned here
};

struct MeshComponent {
    uint32_t draw_item = 0;  // index into the RenderScene's draw items
};

struct MaterialComponent {
    uint32_t material_index = 0;
};

struct ColliderComponent {
    float sphere_radius = 0.0f;  // 0 == use the world's default sphere proxy
};

struct SensorComponent {
    uint32_t sensor_slot = 0;
};

struct ForceElementComponent {
    uint32_t element_slot = 0;
};

struct CameraComponent {
    float fov_degrees = 90.0f;
    float near_plane = 0.1f;
    float far_plane = 1000.0f;
    bool active = true;
};

struct BehaviorComponent {
    uint32_t behavior_index = 0;  // into the BehaviorRegistry (Task 9)
};

struct FluidComponent {
    // RESERVED (stage 5, Task 7; kComponentTypes): SPH, PHY-4, will define it. Declared here so
    // the type id is frozen now -- discovering it later would mean renumbering the enum, which
    // is the one edit this design forbids. Nothing can attach one: ObjectGraph::attach<T> does
    // not compile for a reserved type and the loader refuses it with `unavailable`.
    float rest_density = 1000.0f;
    float stiffness = 0.0f;
    float viscosity = 0.0f;
};

// ---------------------------------------------------------------------------
// Type -> id mapping. One specialization per type; a type with no
// specialization fails to COMPILE rather than silently taking a default id.
// ---------------------------------------------------------------------------
template <class T>
struct ComponentTraits;

#define SPADE_COMPONENT_TRAIT(Type, Enum, Name)                      \
    template <>                                                      \
    struct ComponentTraits<Type> {                                   \
        static constexpr ComponentTypeId id = ComponentTypeId::Enum; \
        static constexpr std::string_view name = Name;               \
    }

SPADE_COMPONENT_TRAIT(TransformComponent, transform, "transform");
SPADE_COMPONENT_TRAIT(BodyComponent, body, "body");
SPADE_COMPONENT_TRAIT(MeshComponent, mesh, "mesh");
SPADE_COMPONENT_TRAIT(MaterialComponent, material, "material");
SPADE_COMPONENT_TRAIT(ColliderComponent, collider, "collider");
SPADE_COMPONENT_TRAIT(SensorComponent, sensor, "sensor");
SPADE_COMPONENT_TRAIT(ForceElementComponent, force_element, "force_element");
SPADE_COMPONENT_TRAIT(CameraComponent, camera, "camera");
SPADE_COMPONENT_TRAIT(BehaviorComponent, behavior, "behavior");
SPADE_COMPONENT_TRAIT(FluidComponent, fluid, "fluid");

#undef SPADE_COMPONENT_TRAIT

template <class T>
[[nodiscard]] constexpr ComponentTypeId component_type_id() noexcept {
    return ComponentTraits<T>::id;
}

// AVAILABILITY (module API stage 5, Task 7; spec section 3): which of the ten types can
// be built. A type no module defines is RESERVED: its id and its name are frozen (SL5), but
// ObjectGraph::attach<T> does not compile for it and the loader refuses it with `unavailable`,
// naming the reason. Static for now (the graph is not in stepping, spec section 11); per module
// set it becomes when objects join stepping. One table, in id order, is the only site that
// holds a type's name, its availability and its reason.
enum class ComponentAvailability : uint8_t { available = 0, reserved = 1 };

struct ComponentTypeInfo {
    ComponentTypeId id;
    std::string_view name;  // the serialization key
    ComponentAvailability availability;
    std::string_view reason;  // reserved only: why, and what will provide it
};

inline constexpr ComponentTypeInfo kComponentTypes[kComponentTypeCount] = {
    {ComponentTypeId::transform, "transform", ComponentAvailability::available, ""},
    {ComponentTypeId::body, "body", ComponentAvailability::available, ""},
    {ComponentTypeId::mesh, "mesh", ComponentAvailability::available, ""},
    {ComponentTypeId::material, "material", ComponentAvailability::available, ""},
    {ComponentTypeId::collider, "collider", ComponentAvailability::available, ""},
    {ComponentTypeId::sensor, "sensor", ComponentAvailability::available, ""},
    {ComponentTypeId::force_element, "force_element", ComponentAvailability::available, ""},
    {ComponentTypeId::camera, "camera", ComponentAvailability::available, ""},
    {ComponentTypeId::behavior, "behavior", ComponentAvailability::available, ""},
    {ComponentTypeId::fluid, "fluid", ComponentAvailability::reserved, "SPH, PHY-4"},
};

// Whether a type can be built: ObjectGraph::attach<T> static_asserts it, so a reserved type
// does not compile into an attach.
template <class T>
[[nodiscard]] consteval bool component_available() noexcept {
    return kComponentTypes[static_cast<uint32_t>(ComponentTraits<T>::id)].availability ==
           ComponentAvailability::available;
}

// Every type, in id order: what every enumerator (the loader, the editor's menu) reads.
[[nodiscard]] std::span<const ComponentTypeInfo> component_types() noexcept;

[[nodiscard]] std::string_view component_type_name(ComponentTypeId id) noexcept;
[[nodiscard]] std::optional<ComponentTypeId> component_type_id_from_name(
    std::string_view name) noexcept;

}  // namespace spade::objects

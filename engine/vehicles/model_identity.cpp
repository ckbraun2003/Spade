// vehicles/model_identity.cpp -- see model_identity.hpp for the byte spelling.
#include "vehicles/model_identity.hpp"

#include <bit>
#include <cstddef>
#include <string_view>

#include "core/rng.hpp"

namespace spade::vehicles {
namespace {

class Fold {
public:
    void byte(uint8_t b) noexcept {
        h_ ^= b;
        h_ *= rng::kFnv1aPrime;
    }
    void u32(uint32_t v) noexcept {
        for (int i = 0; i < 4; ++i) byte(static_cast<uint8_t>(v >> (8 * i)));
    }
    void u64(uint64_t v) noexcept {
        for (int i = 0; i < 8; ++i) byte(static_cast<uint8_t>(v >> (8 * i)));
    }
    void f32(float v) noexcept { u32(std::bit_cast<uint32_t>(v)); }
    void vec3(const glm::vec3& v) noexcept {
        f32(v.x);
        f32(v.y);
        f32(v.z);
    }
    void quat(const glm::quat& q) noexcept {
        f32(q.w);
        f32(q.x);
        f32(q.y);
        f32(q.z);
    }
    void text(std::string_view s) noexcept {
        for (const char c : s) byte(static_cast<uint8_t>(c));
        byte(0);
    }
    // parts_identity's strings: the length (u32), then the bytes.
    void sized_text(std::string_view s) noexcept {
        count(s.size());
        for (const char c : s) byte(static_cast<uint8_t>(c));
    }
    void count(std::size_t n) noexcept { u32(static_cast<uint32_t>(n)); }
    [[nodiscard]] uint64_t value() const noexcept { return h_; }

private:
    uint64_t h_ = rng::kFnv1aOffsetBasis;
};

}  // namespace

uint64_t model_identity(const ModelType& m) noexcept {
    Fold f;
    f.text(m.name);
    f.u32(m.version);
    f.u32(m.param_schema_id);
    f.f32(m.body.mass);
    f.vec3(m.body.inertia_diag);
    f.f32(m.proxy_radius);
    f.quat(m.design_to_principal);
    f.vec3(m.com_offset);
    f.count(m.rotors.size());
    for (const RotorDesc& r : m.rotors) {
        f.vec3(r.local_pos);
        f.quat(r.local_orient);
        f.f32(r.spin_dir);
        f.f32(r.tau);
        f.f32(r.radius);
        f.f32(r.thrust_coeff);
        f.f32(r.torque_coeff);
    }
    f.count(m.drag_bodies.size());
    for (const DragBodyDesc& d : m.drag_bodies) {
        f.u32(d.mode);
        f.f32(d.area);
        f.vec3(d.coeffs);
        f.vec3(d.local_pos);
        f.quat(d.local_orient);
    }
    f.count(m.imu_mounts.size());
    for (const ImuMountDesc& i : m.imu_mounts) {
        f.vec3(i.mount_pos);
        f.quat(i.mount_orient);
        f.u32(i.rate_divider);
        f.f32(i.sigma_a);
        f.f32(i.sigma_g);
        f.f32(i.sigma_ba);
        f.f32(i.sigma_bg);
    }
    return f.value();
}

uint64_t model_registry_identity(std::span<const ModelType> models) noexcept {
    Fold f;
    f.count(models.size());
    for (const ModelType& m : models) {
        f.u64(model_identity(m));
    }
    return f.value();
}

uint64_t parts_identity(const ModelType& m) noexcept {
    if (m.parts.empty()) return kNoPartsIdentity;
    Fold f;
    f.byte(0xFF);
    for (const char c : std::string_view("parts_identity")) f.byte(static_cast<uint8_t>(c));
    f.count(m.parts.size());
    for (const PartDesc& p : m.parts) {
        f.sized_text(p.name);
        f.sized_text(p.role);
        f.u32(static_cast<uint32_t>(p.attach));
        f.u32(p.rotor);
        f.vec3(p.mount_pos);
        f.quat(p.mount_orient);
        if (p.mesh != nullptr) {
            f.u32(1);
            f.u64(p.mesh->content_hash());
        } else {
            f.u32(0);
            f.count(p.shape.nodes.size());
            for (const SdfNode& n : p.shape.nodes) {
                f.u32(n.kind);
                f.u32(n.op);
                f.u32(n.transform);
                f.f32(n.params.x);
                f.f32(n.params.y);
                f.f32(n.params.z);
                f.f32(n.params.w);
            }
            f.count(p.shape.transforms.size());
            for (const SdfTransform& t : p.shape.transforms) {
                for (int c = 0; c < 4; ++c) {
                    for (int r = 0; r < 4; ++r) f.f32(t.world_to_local[c][r]);
                }
                f.f32(t.scale);
            }
        }
        f.u32(static_cast<uint32_t>(p.surface.kind));
        f.f32(p.surface.inv_permeability);
        f.f32(p.surface.inertial_coeff);
        f.f32(p.min_feature);
    }
    return f.value();
}

// RED: a stub, so the tests compile and fail on behaviour.
uint64_t propulsion_identity(const PropulsionChain&) noexcept { return 0; }

}  // namespace spade::vehicles

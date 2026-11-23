// camera_host.cpp -- 3.3's CameraHost (see camera_host.hpp for what it is and why it is not a Simulation member).
#include "sim/camera_host.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

#include "physics/blade_azimuth.hpp"
#include "sensors/camera.hpp"
#include "sensors/rings.hpp"

namespace spade {

namespace {

[[nodiscard]] Error invalid(std::string context) { return Error{Code::invalid_argument, std::move(context)}; }

constexpr uint32_t kStateMagic = 0x484D4143u;  // "CAMH"
constexpr uint32_t kStateVersion = 1;
constexpr uint32_t kMaxBladeCount = 16;
constexpr uint32_t kMaxPolePairs = 64;

[[nodiscard]] bool finite3(const glm::vec3& v) noexcept { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// The state's byte stream: fixed-width little-endian-by-host fields, read back with every length checked.
struct Writer {
    std::vector<std::byte> bytes;
    template <class T>
    void put(const T& v) {
        const std::size_t at = bytes.size();
        bytes.resize(at + sizeof(T));
        std::memcpy(bytes.data() + at, &v, sizeof(T));
    }
};

struct Reader {
    std::span<const std::byte> bytes;
    std::size_t at = 0;
    bool ok = true;
    template <class T>
    T get() {
        T v{};
        if (!ok || bytes.size() - at < sizeof(T)) {
            ok = false;
            return v;
        }
        std::memcpy(&v, bytes.data() + at, sizeof(T));
        at += sizeof(T);
        return v;
    }
    [[nodiscard]] std::size_t left() const noexcept { return bytes.size() - at; }
};

}  // namespace

struct CameraHost::Impl {
    Simulation* sim = nullptr;
    uint64_t expected_tick = 0;

    struct Cam {
        RowRef row;
        sensors::SampleIndex since = 0;
        int64_t exposure_ns = 0;
    };
    std::vector<Cam> cams;

    std::vector<float> psi;        // by GLOBAL rotor slot
    std::vector<RotorInfo> info;   // likewise

    struct Tagged {
        BodyRef body;
        bool has_label = false;
        uint32_t label = 0;
        std::vector<glm::vec3> points;
    };
    std::vector<Tagged> bodies;

    [[nodiscard]] Tagged* find(BodyRef b) {
        for (Tagged& t : bodies) {
            if (t.body == b) return &t;
        }
        return nullptr;
    }
    [[nodiscard]] const Tagged* find(BodyRef b) const {
        for (const Tagged& t : bodies) {
            if (t.body == b) return &t;
        }
        return nullptr;
    }
    [[nodiscard]] Tagged& find_or_add(BodyRef b) {
        if (Tagged* t = find(b)) return *t;
        bodies.push_back(Tagged{.body = b});
        return bodies.back();
    }
    [[nodiscard]] Result<void> check_body(BodyRef b) const {
        if (b.is_null()) return std::unexpected(invalid("camera host: a null body"));
        const Result<const BodyState*> state = sim->body(b);
        if (!state) return std::unexpected(invalid("camera host: no such body (" + state.error().context + ")"));
        return {};
    }
    void fit_rotors() {
        const auto rows = sim->arenas().array(sim->rotors_array());
        const std::size_t n = rows ? rows->size() : 0;
        if (psi.size() < n) psi.resize(n, 0.0f);
        if (info.size() < n) info.resize(n);
    }
};

Result<CameraHost> CameraHost::open(Simulation& sim) {
    CameraHost host;
    host.impl_ = std::make_shared<Impl>();
    host.impl_->sim = &sim;
    host.impl_->expected_tick = sim.tick().value;
    host.impl_->fit_rotors();
    return host;
}

Result<uint32_t> CameraHost::add_camera(BodyRef body, const CameraSpec& spec) {
    Impl& m = *impl_;
    const uint32_t substeps = m.sim->substeps();
    if (spec.rate_divider == 0 || spec.rate_divider % substeps != 0) {
        return std::unexpected(invalid("camera: rate_divider " + std::to_string(spec.rate_divider) + " is not a whole number of ticks (a tick is " +
                                       std::to_string(substeps) + " substeps)"));
    }
    if (!std::isfinite(spec.exposure_s) || spec.exposure_s < 0.0) return std::unexpected(invalid("camera: exposure_s must be finite and at least 0"));
    const double period_s = static_cast<double>(spec.rate_divider) * static_cast<double>(m.sim->substep_dt_ns()) * 1e-9;
    if (spec.exposure_s > period_s) {
        return std::unexpected(invalid("camera: exposure_s " + std::to_string(spec.exposure_s) + " is longer than the camera's period " + std::to_string(period_s) + " s"));
    }
    sensors::CameraSensorSpawn s;
    s.mount_pos = spec.mount_pos_body_m;
    s.mount_orient = spec.mount_orient;
    s.rate_divider = spec.rate_divider;
    s.shutter_fraction = static_cast<float>(spec.exposure_s / period_s);
    s.lens_kind = spec.lens_kind;
    s.width = spec.width;
    s.height = spec.height;
    s.fx = spec.fx;
    s.fy = spec.fy;
    s.cx = spec.cx;
    s.cy = spec.cy;
    s.k1 = spec.k[0];
    s.k2 = spec.k[1];
    s.k3 = spec.k[2];
    s.k4 = spec.k[3];
    s.channels = spec.channels;
    const Result<RowRef> row = m.sim->attach_row(body, "camera_sensors", s);
    if (!row) return std::unexpected(row.error());
    if (Result<void> flushed = m.sim->flush_structural(); !flushed) return std::unexpected(flushed.error());
    Impl::Cam cam;
    cam.row = *row;
    cam.exposure_ns = std::llround(spec.exposure_s * 1e9);
    m.cams.push_back(cam);
    return static_cast<uint32_t>(m.cams.size() - 1);
}

Result<void> CameraHost::reset_camera(uint32_t camera, const CameraReset& reset) {
    Impl& m = *impl_;
    if (camera >= m.cams.size()) return std::unexpected(invalid("camera host: no camera " + std::to_string(camera)));
    const RowRef ref = m.cams[camera].row;
    const auto rows = m.sim->module_rows<sensors::CameraSensorRow>("camera_sensors", ref.world_index);
    if (!rows) return std::unexpected(rows.error());
    std::vector<sensors::CameraSensorRow> copy(rows->begin(), rows->end());
    if (ref.slot >= copy.size()) return std::unexpected(invalid("camera host: the camera's row is gone"));
    sensors::CameraSensorRow& row = copy[ref.slot];
    row.mount_orient = reset.mount_orient;
    if (reset.set_lens) {
        row.lens_kind = reset.lens_kind;
        row.fx = reset.fx;
        row.fy = reset.fy;
        row.cx = reset.cx;
        row.cy = reset.cy;
        row.k1 = reset.k[0];
        row.k2 = reset.k[1];
        row.k3 = reset.k[2];
        row.k4 = reset.k[3];
    }
    const double length = std::sqrt(static_cast<double>(reset.mount_orient.w) * reset.mount_orient.w + static_cast<double>(reset.mount_orient.x) * reset.mount_orient.x +
                                    static_cast<double>(reset.mount_orient.y) * reset.mount_orient.y + static_cast<double>(reset.mount_orient.z) * reset.mount_orient.z);
    if (!(std::fabs(length - 1.0) <= 1e-3)) return std::unexpected(invalid("camera host: mount_orient must be a unit quaternion"));
    // The row's own rules, naming the field (keypoint_groups is the world's: the reset never changes the channels, so any value passes it).
    if (const std::string issue = sensors::camera_row_issue(row, std::numeric_limits<uint32_t>::max()); !issue.empty()) {
        return std::unexpected(invalid("camera host: reset: " + issue));
    }
    return m.sim->write_module_rows("camera_sensors", ref.world_index, std::as_bytes(std::span<const sensors::CameraSensorRow>(copy)));
}

Result<void> CameraHost::set_label(BodyRef body, uint32_t label) {
    Impl& m = *impl_;
    if (Result<void> ok = m.check_body(body); !ok) return ok;
    Impl::Tagged& t = m.find_or_add(body);
    t.has_label = true;
    t.label = label;
    return {};
}

Result<void> CameraHost::set_keypoints(BodyRef body, std::span<const glm::vec3> local_points) {
    Impl& m = *impl_;
    if (Result<void> ok = m.check_body(body); !ok) return ok;
    for (std::size_t i = 0; i < local_points.size(); ++i) {
        if (!finite3(local_points[i])) return std::unexpected(invalid("camera host: keypoint " + std::to_string(i) + " is not finite"));
    }
    m.find_or_add(body).points.assign(local_points.begin(), local_points.end());
    return {};
}

Result<uint32_t> CameraHost::label_of(BodyRef body) const {
    if (Result<void> ok = impl_->check_body(body); !ok) return std::unexpected(ok.error());
    const Impl::Tagged* t = impl_->find(body);
    return t && t->has_label ? t->label : 0u;
}

Result<std::span<const glm::vec3>> CameraHost::keypoints_of(BodyRef body) const {
    if (Result<void> ok = impl_->check_body(body); !ok) return std::unexpected(ok.error());
    const Impl::Tagged* t = impl_->find(body);
    if (!t) return std::span<const glm::vec3>{};
    return std::span<const glm::vec3>(t->points);
}

Result<StepReport> CameraHost::step(uint64_t ticks) {
    Impl& m = *impl_;
    if (m.sim->tick().value != m.expected_tick) {
        return std::unexpected(invalid("camera host: the simulation was restored without the camera host's state (the simulation is at tick " +
                                       std::to_string(m.sim->tick().value) + ", the host expects tick " + std::to_string(m.expected_tick) +
                                       "); load() the state saved beside the snapshot"));
    }
    StepReport report;
    report.ticks = ticks;
    const uint64_t dt_ns = m.sim->dt_ns();
    const float h = static_cast<float>(static_cast<double>(dt_ns) * 1e-9);
    std::array<sensors::CameraFrameDue, sensors::kRingDepth> out{};
    for (uint64_t t = 0; t < ticks; ++t) {
        if (Result<void> stepped = m.sim->step(1); !stepped) return std::unexpected(stepped.error());
        m.expected_tick = m.sim->tick().value;

        m.fit_rotors();
        if (const auto rows = m.sim->arenas().array(m.sim->rotors_array()); rows && !rows->empty()) {
            (void)physics::advance_blade_azimuths(std::span<float>(m.psi).first(rows->size()), std::span<const vehicles::RotorRow>(rows->data(), rows->size()), h);
        }

        for (uint32_t c = 0; c < m.cams.size(); ++c) {
            Impl::Cam& cam = m.cams[c];
            const auto cams = m.sim->module_rows<sensors::CameraSensorRow>("camera_sensors", cam.row.world_index);
            const auto ring = m.sim->module_rows<sensors::CameraFrameDue>("camera_frames", cam.row.world_index);
            if (!cams || !ring) return std::unexpected(invalid("camera host: the camera arrays cannot be read"));
            const auto window = std::span<const sensors::CameraFrameDue>(*ring).subspan(std::size_t{cam.row.slot} * sensors::kRingDepth, sensors::kRingDepth);
            const auto polled = sensors::ring_poll<sensors::CameraFrameDue>(window, (*cams)[cam.row.slot].last_index, cam.since, out);
            cam.since = polled.next_since;
            for (const sensors::CameraFrameDue& f : polled.samples) {
                report.due.push_back(DueFrame{.camera = c,
                                              .frame_seq = f.index,
                                              .tick = f.tick,
                                              .capture_time_ns = static_cast<int64_t>((f.tick + 1) * dt_ns) - cam.exposure_ns / 2});
            }
        }
    }
    return report;
}

Result<RotorReading> CameraHost::rotors(const VehicleRef& vehicle) const {
    Impl& m = *impl_;
    m.fit_rotors();
    RotorReading r;
    for (uint32_t i = 0; i < vehicle.rotor_count; ++i) {
        const Result<const vehicles::RotorRow*> row = m.sim->rotor(vehicle, i);
        if (!row) return std::unexpected(row.error());
        const uint32_t slot = vehicle.rotor_slots[i];
        r.omega.push_back((*row)->omega);
        r.psi.push_back(slot < m.psi.size() ? m.psi[slot] : 0.0f);
        r.blade_count.push_back(slot < m.info.size() ? m.info[slot].blade_count : 0u);
        r.pole_pairs.push_back(slot < m.info.size() ? m.info[slot].pole_pairs : 0u);
    }
    return r;
}

Result<void> CameraHost::set_rotor_info(const VehicleRef& vehicle, std::span<const RotorInfo> info) {
    Impl& m = *impl_;
    if (info.size() != vehicle.rotor_count) {
        return std::unexpected(invalid("camera host: set_rotor_info gives " + std::to_string(info.size()) + " entries for a vehicle with " +
                                       std::to_string(vehicle.rotor_count) + " rotors"));
    }
    for (uint32_t i = 0; i < vehicle.rotor_count; ++i) {
        if (const Result<const vehicles::RotorRow*> row = m.sim->rotor(vehicle, i); !row) return std::unexpected(row.error());
        if (info[i].blade_count > kMaxBladeCount) {
            return std::unexpected(invalid("camera host: rotor " + std::to_string(i) + " blade_count " + std::to_string(info[i].blade_count) + " is above " +
                                           std::to_string(kMaxBladeCount)));
        }
        if (info[i].pole_pairs > kMaxPolePairs) {
            return std::unexpected(invalid("camera host: rotor " + std::to_string(i) + " pole_pairs " + std::to_string(info[i].pole_pairs) + " is above " +
                                           std::to_string(kMaxPolePairs)));
        }
    }
    m.fit_rotors();
    for (uint32_t i = 0; i < vehicle.rotor_count; ++i) {
        const uint32_t slot = vehicle.rotor_slots[i];
        if (slot >= m.info.size()) m.info.resize(slot + 1);
        m.info[slot] = info[i];
    }
    return {};
}

std::vector<std::byte> CameraHost::save() const {
    const Impl& m = *impl_;
    Writer w;
    w.put(kStateMagic);
    w.put(kStateVersion);
    w.put(m.expected_tick);
    w.put(static_cast<uint32_t>(m.cams.size()));
    for (const Impl::Cam& c : m.cams) w.put(c.since);
    w.put(static_cast<uint32_t>(m.psi.size()));
    for (const float p : m.psi) w.put(p);
    w.put(static_cast<uint32_t>(m.info.size()));
    for (const RotorInfo& i : m.info) {
        w.put(i.blade_count);
        w.put(i.pole_pairs);
    }
    w.put(static_cast<uint32_t>(m.bodies.size()));
    for (const Impl::Tagged& t : m.bodies) {
        w.put(t.body.world_index);
        w.put(t.body.slot);
        w.put(t.body.generation);
        w.put(static_cast<uint32_t>(t.has_label ? 1u : 0u));
        w.put(t.label);
        w.put(static_cast<uint32_t>(t.points.size()));
        for (const glm::vec3& p : t.points) {
            w.put(p.x);
            w.put(p.y);
            w.put(p.z);
        }
    }
    return std::move(w.bytes);
}

Result<void> CameraHost::load(std::span<const std::byte> bytes) {
    Impl& m = *impl_;
    Reader r{bytes};
    if (r.get<uint32_t>() != kStateMagic) return std::unexpected(invalid("camera host: load: not a camera host state"));
    if (r.get<uint32_t>() != kStateVersion) return std::unexpected(invalid("camera host: load: a state of another version"));
    const uint64_t tick = r.get<uint64_t>();
    const uint32_t cam_count = r.get<uint32_t>();
    if (!r.ok || cam_count != m.cams.size()) {
        return std::unexpected(invalid("camera host: load: the state has " + std::to_string(cam_count) + " cameras, this host has " + std::to_string(m.cams.size())));
    }
    std::vector<sensors::SampleIndex> since(cam_count);
    for (auto& s : since) s = r.get<sensors::SampleIndex>();
    const uint32_t psi_count = r.get<uint32_t>();
    if (!r.ok || r.left() / sizeof(float) < psi_count) return std::unexpected(invalid("camera host: load: the state is cut short (psi)"));
    std::vector<float> psi(psi_count);
    for (float& p : psi) p = r.get<float>();
    const uint32_t info_count = r.get<uint32_t>();
    if (!r.ok || r.left() / (2 * sizeof(uint32_t)) < info_count) return std::unexpected(invalid("camera host: load: the state is cut short (rotor info)"));
    std::vector<RotorInfo> info(info_count);
    for (RotorInfo& i : info) {
        i.blade_count = r.get<uint32_t>();
        i.pole_pairs = r.get<uint32_t>();
    }
    const uint32_t body_count = r.get<uint32_t>();
    std::vector<Impl::Tagged> bodies;
    for (uint32_t b = 0; b < body_count && r.ok; ++b) {
        Impl::Tagged t;
        t.body.world_index = r.get<uint32_t>();
        t.body.slot = r.get<uint32_t>();
        t.body.generation = r.get<uint32_t>();
        t.has_label = r.get<uint32_t>() != 0u;
        t.label = r.get<uint32_t>();
        const uint32_t points = r.get<uint32_t>();
        if (!r.ok || r.left() / (3 * sizeof(float)) < points) {
            r.ok = false;
            break;
        }
        for (uint32_t p = 0; p < points; ++p) {
            glm::vec3 v;
            v.x = r.get<float>();
            v.y = r.get<float>();
            v.z = r.get<float>();
            t.points.push_back(v);
        }
        bodies.push_back(std::move(t));
    }
    if (!r.ok || r.left() != 0) return std::unexpected(invalid("camera host: load: the state is cut short or has bytes left over"));
    for (const float p : psi) {
        if (!(std::isfinite(p) && p >= 0.0f && p < physics::kAzimuthTwoPi)) return std::unexpected(invalid("camera host: load: a blade azimuth is out of [0, 2 pi)"));
    }
    // All read and checked: only now does anything change.
    m.expected_tick = tick;
    for (uint32_t c = 0; c < cam_count; ++c) m.cams[c].since = since[c];
    m.psi = std::move(psi);
    m.info = std::move(info);
    m.bodies = std::move(bodies);
    m.fit_rotors();
    return {};
}

int64_t CameraHost::sample_time_ns(uint64_t tick) const noexcept { return static_cast<int64_t>((tick + 1) * impl_->sim->dt_ns()); }

}  // namespace spade

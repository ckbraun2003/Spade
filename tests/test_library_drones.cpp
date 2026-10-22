// The default library's six drones (Task L5 of docs/design/core/plans/
// 2026-10-21-default-drone-library-plan.md): phantom3_class, iris_plus_class,
// fpv_5inch, crazyflie_class, basic_quad_6inch and basic_quad_competition,
// built in tests/library_drones.hpp from the library's part files and one
// inputs table each (TD-9).
//
// What is pinned here:
//   - THE GOLDENS, and their GENERATOR. Each committed assets/library/drones/
//     <name>.model.yaml is model_to_yaml() of compile_airframe(spec).model,
//     its propulsion set to airframe_propulsion_chain(spec) (test_library_
//     quad.cpp's flow), byte for byte; <name>.sources.yaml beside it is
//     model_sources_to_yaml() of the drone's sidecar: the model and its four
//     part files pinned by hash, its inputs table, its checks, what it does
//     not carry, its assumptions and its caveat. With SPADE_LIBRARY_WRITE set,
//     <Drone>FileIsItsCompiledSpec WRITES both into the source tree instead
//     of comparing, and passes, saying so. They are only ever written that
//     way, never by hand.
//   - EVERY INPUT CITES A SOURCE (the L5 draft's section 5): the committed
//     sidecar parses (1); it pins the committed model (2) and the drone's
//     part files, each an index entry that loads, and the spec's blocks are
//     the files' field for field, but for comp's kappa-derived torque, which
//     cites kappa (3); its inputs are the drone's table exactly (4), each
//     value the spec's or the params' bit for bit (5); each citation passes
//     check_source_ref() (6); none cites kat_datapack, and no fitted grade
//     cites withdrawn work (7); a basic quad's defaults used unchanged are
//     assumed with basic_quad_assumptions()' note verbatim, every one of its
//     fields appears (8), and its assumptions hold section 4's kappa rule and
//     the tri-blade line (9).
//   - THE CHECKS, as Physics ruled them ("L5's checks", 2026-10-22), each
//     against the committed model through its index entry, its reference and
//     band the drone's own (library_drones.hpp, written to the sidecar's
//     `checks`). A report-only value goes through RecordProperty and printf,
//     never EXPECT.
//
// LINE ENDINGS. Both writers write LF, and the comparisons read the files in
// binary; assets/.gitattributes gives *.model.yaml and *.sources.yaml `text
// eol=lf`.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "library_drones.hpp"
#include "scene/library.hpp"
#include "scene/library_sources.hpp"
#include "scene/model_file.hpp"
#include "scene/model_sources.hpp"
#include "scene/part_file.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/basic_quad.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/propulsion_flags.hpp"
#include "vehicles/propulsion_steady.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;
namespace ld = spade::test::library_drones;
namespace pf = spade::vehicles::propulsion_flags;
using sc::Grade;
using sc::InputValue;
using sc::Library;
using sc::LibraryEntry;
using sc::LibraryKind;
using sc::ModelCheck;
using sc::ModelFile;
using sc::ModelInput;
using sc::ModelSources;
using sc::PartFile;
using sc::SourceRef;
using sc::SourceRegistry;
using spade::vehicles::AirframeSpec;
using spade::vehicles::BasicQuadParams;
using spade::vehicles::MountPose;
using spade::vehicles::PartInertia;
using spade::vehicles::SteadyPoint;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                       << r.error().context;
}
#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

[[nodiscard]] std::uint64_t bits(double v) { return std::bit_cast<std::uint64_t>(v); }

[[nodiscard]] bool read_bytes(const fs::path& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

// The committed library, in the SOURCE tree: the generator writes here.
[[nodiscard]] fs::path library_dir() { return fs::path(SPADE_ASSETS_DIR) / "library"; }

// SPADE_LIBRARY_WRITE set: write the drones' files instead of comparing them.
// A read-only null check before any thread starts, as env_gate_is_set() in
// test_fp32_math.cpp.
[[nodiscard]] bool library_write_asked() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_LIBRARY_WRITE") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// The line holding byte `at` of `text` (or "<end of file>").
[[nodiscard]] std::string line_at(const std::string& text, std::size_t at) {
    if (at >= text.size()) return "<end of file>";
    std::size_t from = 0;
    if (at > 0) {
        const std::size_t newline = text.rfind('\n', at - 1);
        if (newline != std::string::npos) from = newline + 1;
    }
    const std::size_t end = text.find('\n', at);
    return text.substr(from, (end == std::string::npos ? text.size() : end) - from);
}

// Why the committed bytes are not the generator's, and what to do about it.
[[nodiscard]] std::string mismatch(const fs::path& path, const std::string& committed, const std::string& generated,
                                   std::string_view test) {
    std::size_t i = 0;
    while (i < committed.size() && i < generated.size() && committed[i] == generated[i]) ++i;
    std::size_t line = 1;
    for (std::size_t j = 0; j < i; ++j) {
        if (committed[j] == '\n') ++line;
    }
    std::string out = spade::path_text(path) + " (" + std::to_string(committed.size()) +
                      " bytes) is not the generator's text (" + std::to_string(generated.size()) +
                      " bytes): first difference at byte " + std::to_string(i) + ", line " + std::to_string(line) +
                      ".\n  committed: " + line_at(committed, i) + "\n  generated: " + line_at(generated, i) + "\n";
    if (committed.find('\r') != std::string::npos) {
        out += "The committed file holds CR bytes: a checkout converted its line endings. assets/.gitattributes "
               "must give *.model.yaml and *.sources.yaml `text eol=lf` (check with `git check-attr -a` on the "
               "path).\n";
    }
    out += "The file is generated from tests/library_drones.hpp and the library's part files; never edit it by "
           "hand. To move it: (1) name the cause (TD-1): the drone's table, a part file, the airframe compile, "
           "basic_quad or a file format; (2) regenerate: set SPADE_LIBRARY_WRITE=1 and run ctest -R "
           "LibraryDrones." + std::string(test) + "; (3) it is final only once a fresh (-NoSeed) Docker gcc leg "
           "reproduces it (TD-12).";
    return out;
}

[[nodiscard]] bool contains(const std::vector<std::string>& list, std::string_view item) {
    return std::find(list.begin(), list.end(), item) != list.end();
}

[[nodiscard]] std::string text_of(const InputValue& value) {
    char buf[40];
    if (const double* x = std::get_if<double>(&value)) {
        std::snprintf(buf, sizeof(buf), "%.17g", *x);
        return buf;
    }
    std::string out = "[";
    for (const double x : std::get<std::vector<double>>(value)) {
        std::snprintf(buf, sizeof(buf), "%.17g", x);
        out += (out.size() > 1 ? ", " : "") + std::string(buf);
    }
    return out + "]";
}

// The library as committed, and its registry. The index alone, its dir set:
// load_library() refuses an entry whose file is missing, and the generator is
// what writes the drones' model files (test_library_thumbnails.cpp's
// the_index()). The committed files themselves are checked byte for byte below.
struct Committed {
    Library library{};
    SourceRegistry registry{};
};

[[nodiscard]] spade::Result<Committed> committed_library() {
    std::string text;
    if (!read_bytes(library_dir() / "library.yaml", text)) {
        return std::unexpected(spade::Error{spade::Code::io_error, "library.yaml can't be read"});
    }
    spade::Result<Library> library = sc::library_from_yaml(text);
    if (!library) return std::unexpected(library.error());
    library->dir = library_dir();
    spade::Result<SourceRegistry> registry = sc::load_library_sources(*library);
    if (!registry) return std::unexpected(registry.error());
    return Committed{std::move(*library), std::move(*registry)};
}

[[nodiscard]] const LibraryEntry* entry_for(const Library& library, std::string_view file) {
    for (const LibraryEntry& e : library.entries) {
        if (e.file == file) return &e;
    }
    return nullptr;
}

// The committed model of the drone, through its index entry.
[[nodiscard]] spade::Result<ModelFile> drone_model(const Library& library, std::string_view stem) {
    const std::string file = ld::model_file(stem);
    const LibraryEntry* entry = entry_for(library, file);
    if (entry == nullptr) {
        return std::unexpected(
            spade::Error{spade::Code::invalid_argument, "library.yaml has no entry whose file is \"" + file + "\""});
    }
    return sc::load_library_model(library, *entry);
}

// The drone's check of `test`: its banded one, or its report-only ones.
[[nodiscard]] const ModelCheck* banded_check(const ld::DroneNotes& notes, std::string_view test) {
    for (const ModelCheck& c : notes.checks) {
        if (c.test == test && !c.report_only && c.band.has_value()) return &c;
    }
    return nullptr;
}

[[nodiscard]] std::vector<const ModelCheck*> report_checks(const ld::DroneNotes& notes, std::string_view test) {
    std::vector<const ModelCheck*> out;
    for (const ModelCheck& c : notes.checks) {
        if (c.test == test && c.report_only) out.push_back(&c);
    }
    return out;
}

// The check's relative error, (value - reference) / reference, held to its band.
void expect_in_band(const ModelCheck& check, double relative, std::string_view what) {
    ASSERT_TRUE(check.band.has_value()) << check.test << " has no band";
    EXPECT_GE(relative, (*check.band)[0]) << what << ": " << 100.0 * relative << "% is below the band";
    EXPECT_LE(relative, (*check.band)[1]) << what << ": " << 100.0 * relative << "% is above the band";
}

// A report-only number: printed and recorded, never EXPECTed.
void report(const std::string& key, double value, std::string_view what) {
    std::printf("[ report   ] %s = %.6g (%.*s)\n", key.c_str(), value, static_cast<int>(what.size()), what.data());
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.6g", value);
    testing::Test::RecordProperty(key, std::string(buf));
}

[[nodiscard]] double rpm_of(double omega) { return omega * 60.0 / (2.0 * std::numbers::pi); }
[[nodiscard]] double omega_of(double rpm) { return rpm * 2.0 * std::numbers::pi / 60.0; }

// The rotors' summed k_T (float, as the file holds it).
[[nodiscard]] double thrust_coeff_sum(const spade::vehicles::ModelType& m) {
    double k = 0.0;
    for (const spade::vehicles::RotorDesc& r : m.rotors) k += static_cast<double>(r.thrust_coeff);
    return k;
}

// Every bit a propulsion point sets when a limit holds or the solver fails.
constexpr std::uint32_t kLimitFlags = pf::duty_clamped | pf::current_limited | pf::speed_clamped |
                                      pf::battery_current_limited | pf::battery_cutoff | pf::battery_empty |
                                      pf::unreachable | pf::no_bracket | pf::esc_total_limited;

// The state of charge at which the chain's LOADED bus reads `volts` at `duty`,
// by bisection, since the bus voltage rises with the state of charge
// (report-only use). Outside the pack's range it lands at the nearer end, so
// the caller reports the bus voltage it reached.
[[nodiscard]] double soc_at_loaded_bus(const spade::vehicles::PropulsionChain& chain, double duty, double density,
                                       double volts) {
    double lo = 0.0;
    double hi = 1.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        const SteadyPoint p = spade::vehicles::steady_state_at_duty(chain, duty, density, 0.0, mid);
        if (p.bus_voltage < volts) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

// The standard report of a model's hover and full duty (the draft's "Report
// only" rows): own-mass hover rpm, duty and current, full-duty thrust to
// weight and flags, and the rotor's lag.
void report_hover_and_full_duty(const ld::BuiltDrone& drone, const ModelFile& file, const std::string& key) {
    const spade::vehicles::ModelType& m = file.model;
    if (!m.propulsion.has_value() || m.rotors.empty()) return;
    const AirframeSpec& s = drone.spec;
    const double weight = static_cast<double>(m.body.mass) * s.gravity;
    const double n = static_cast<double>(m.rotors.size());
    const SteadyPoint hover =
        spade::vehicles::steady_state_for_thrust(*m.propulsion, weight / n, s.air_density, 0.0, s.state_of_charge);
    report(key + "_hover_rpm", rpm_of(std::sqrt(weight / thrust_coeff_sum(m))), "own mass, the file's k_T");
    report(key + "_hover_duty", hover.duty, "own mass, the file's chain");
    report(key + "_hover_current", hover.current, "A per motor");
    const SteadyPoint full =
        spade::vehicles::steady_state_at_duty(*m.propulsion, 1.0, s.air_density, 0.0, s.state_of_charge);
    report(key + "_full_thrust_to_weight", n * full.thrust / weight, "full duty, SoC 1");
    report(key + "_full_flags", static_cast<double>(full.flags), "propulsion_flags at full duty");
    report(key + "_tau", static_cast<double>(m.rotors[0].tau), "s, the rotor's fitted lag");
}

// -- What a field path names in a spec or the params (check 5) ----------------

// A field path's segments: "rotors[2].hub.position.x" is rotors[2], hub,
// position, x.
struct Segment {
    std::string_view name{};
    std::optional<std::size_t> index{};
};

[[nodiscard]] std::vector<Segment> segments_of(std::string_view path) {
    std::vector<Segment> out;
    std::size_t from = 0;
    for (;;) {
        const std::size_t dot = path.find('.', from);
        const std::string_view seg = path.substr(from, dot == std::string_view::npos ? dot : dot - from);
        Segment s;
        const std::size_t open = seg.find('[');
        if (open != std::string_view::npos && seg.back() == ']') {
            s.name = seg.substr(0, open);
            std::size_t k = 0;
            for (const char c : seg.substr(open + 1, seg.size() - open - 2)) {
                k = k * 10 + static_cast<std::size_t>(c - '0');
            }
            s.index = k;
        } else {
            s.name = seg;
        }
        out.push_back(s);
        if (dot == std::string_view::npos) return out;
        from = dot + 1;
    }
}

using Found = std::optional<InputValue>;
using Rest = std::span<const Segment>;

[[nodiscard]] bool is(const Rest& rest, std::string_view name) {
    return rest.size() == 1 && !rest[0].index && rest[0].name == name;
}

// A vector field, or one of its components.
[[nodiscard]] Found vector_at(const glm::dvec3& v, Rest rest) {
    if (rest.empty()) return ld::vec_value(v);
    if (is(rest, "x")) return v.x;
    if (is(rest, "y")) return v.y;
    if (is(rest, "z")) return v.z;
    return std::nullopt;
}

[[nodiscard]] Found pose_at(const MountPose& m, Rest rest) {
    if (rest.empty() || rest[0].index) return std::nullopt;
    if (rest[0].name == "position") return vector_at(m.position, rest.subspan(1));
    if (is(rest, "orientation")) return ld::quat_value(m.orientation);
    return std::nullopt;
}

// `rest` under "<block>.mount", for the ESC's and the pack's.
[[nodiscard]] Found mount_at(const MountPose& m, Rest rest) {
    if (rest.empty() || rest[0].index || rest[0].name != "mount") return std::nullopt;
    return pose_at(m, rest.subspan(1));
}

[[nodiscard]] Found spec_value(const AirframeSpec& s, std::string_view path) {
    const std::vector<Segment> all = segments_of(path);
    const Segment& head = all.front();
    const Rest rest = Rest(all).subspan(1);
    if (!head.index) {
        if (rest.empty()) {
            if (head.name == "air_density") return s.air_density;
            if (head.name == "gravity") return s.gravity;
            if (head.name == "state_of_charge") return s.state_of_charge;
            if (head.name == "proxy_radius") return s.proxy_radius;
            return std::nullopt;
        }
        if (head.name == "esc") return mount_at(s.esc.mount, rest);
        if (head.name == "battery") return mount_at(s.battery.mount, rest);
        return std::nullopt;
    }
    const std::size_t k = *head.index;
    if (head.name == "rotors" && k < s.rotors.size()) {
        if (is(rest, "spin_dir")) return s.rotors[k].spin_dir;
        if (!rest.empty() && rest[0].name == "hub") return pose_at(s.rotors[k].hub, rest.subspan(1));
        return std::nullopt;
    }
    if (head.name == "parts" && k < s.parts.size()) {
        const PartInertia& p = s.parts[k];
        if (is(rest, "mass")) return p.mass;
        if (is(rest, "position")) return ld::vec_value(p.position);
        if (is(rest, "orientation")) return ld::quat_value(p.orientation);
        if (is(rest, "size")) return ld::vec_value(p.size);
        if (is(rest, "tensor")) return ld::mat_value(p.tensor);
        return std::nullopt;
    }
    if (head.name == "imus" && k < s.imus.size()) {
        const spade::vehicles::ImuSpec& imu = s.imus[k];
        if (!rest.empty() && rest[0].name == "mount") return pose_at(imu.mount, rest.subspan(1));
        if (is(rest, "rate_divider")) return static_cast<double>(imu.rate_divider);
        if (is(rest, "sigma_a")) return imu.sigma_a;
        if (is(rest, "sigma_g")) return imu.sigma_g;
        if (is(rest, "sigma_ba")) return imu.sigma_ba;
        if (is(rest, "sigma_bg")) return imu.sigma_bg;
        return std::nullopt;
    }
    if (head.name == "drag" && k < s.drag.size()) {
        if (is(rest, "coeffs")) return ld::vec_value(s.drag[k].coeffs);
        if (!rest.empty() && rest[0].name == "mount") return pose_at(s.drag[k].mount, rest.subspan(1));
    }
    return std::nullopt;
}

[[nodiscard]] Found params_value(const BasicQuadParams& p, std::string_view path) {
    const std::vector<Segment> all = segments_of(path);
    const Segment& head = all.front();
    const Rest rest = Rest(all).subspan(1);
    if (head.index) return std::nullopt;
    if (rest.empty()) {
        if (head.name == "half_span_x") return p.half_span_x;
        if (head.name == "half_span_z") return p.half_span_z;
        if (head.name == "hub_height") return p.hub_height;
        if (head.name == "body_size") return ld::vec_value(p.body_size);
        if (head.name == "body_mass") return p.body_mass;
        if (head.name == "arm_radius") return p.arm_radius;
        if (head.name == "arm_mass_per_metre") return p.arm_mass_per_metre;
        if (head.name == "mass" && p.mass) return *p.mass;
        if (head.name == "inertia" && p.inertia) return ld::vec_value(*p.inertia);
        if (head.name == "kappa" && p.kappa) return *p.kappa;
        return std::nullopt;
    }
    if (head.name == "esc") return mount_at(p.esc.mount, rest);
    if (head.name == "battery") return mount_at(p.battery.mount, rest);
    return std::nullopt;
}

// A path's value in what the drone was built from: a basic quad's params
// first, the Crazyflie's fit targets, then the spec.
[[nodiscard]] Found drone_value(const ld::BuiltDrone& drone, std::string_view path) {
    if (drone.params) {
        if (Found v = params_value(*drone.params, path)) return v;
    }
    if (drone.targets) {
        if (path == "mass") return drone.targets->mass;
        if (path == "inertia") return ld::vec_value(drone.targets->inertia);
    }
    return spec_value(drone.spec, path);
}

// The work the test holds as withdrawn (the lead, 2026-10-22: no machine key
// yet): kat_datapack's section 7.2 fit, withdrawn on 2026-10-21 for
// APC-derived inputs.
constexpr std::array<std::string_view, 1> kWithdrawn{"kat_datapack"};

[[nodiscard]] bool is_withdrawn(std::string_view source) {
    return std::find(kWithdrawn.begin(), kWithdrawn.end(), source) != kWithdrawn.end();
}

// -- The generator ---------------------------------------------------------------

struct Generated {
    std::string model{};
    std::string sources{};
};

// The drone's model file text and its sidecar's: the compiled model with its
// chain, and the sidecar pinning that text and the committed part files.
[[nodiscard]] spade::Result<Generated> generate(const ld::BuiltDrone& drone) {
    const auto compiled = spade::vehicles::compile_airframe(drone.spec);
    if (!compiled) return std::unexpected(compiled.error());
    const auto chain = spade::vehicles::airframe_propulsion_chain(drone.spec);
    if (!chain) return std::unexpected(chain.error());
    ModelFile file{compiled->model};
    file.model.propulsion = *chain;
    spade::Result<std::string> model = sc::model_to_yaml(file);
    if (!model) return std::unexpected(model.error());

    ModelSources s;
    s.model = sc::PinnedFile{ld::model_file(drone.def.stem), sc::pinned_file_hash(*model)};
    s.built_by = drone.notes.built_by;
    for (std::size_t i = 0; i < ld::kPartKinds.size(); ++i) {
        const std::string& part = drone.parts.files[i];
        std::string bytes;
        if (!read_bytes(library_dir() / fs::path(part), bytes)) {
            return std::unexpected(spade::Error{spade::Code::io_error, "cannot read the part file '" + part + "'"});
        }
        s.parts.push_back(sc::PinnedPart{ld::kPartKinds[i], part, sc::pinned_file_hash(bytes)});
    }
    s.inputs = drone.inputs.rows;
    s.not_carried = drone.notes.not_carried;
    s.checks = drone.notes.checks;
    s.assumptions = drone.notes.assumptions;
    s.caveat = drone.notes.caveat;
    spade::Result<std::string> sources = sc::model_sources_to_yaml(s);
    if (!sources) return std::unexpected(sources.error());
    return Generated{std::move(*model), std::move(*sources)};
}

// <Drone>FileIsItsCompiledSpec: the golden pair, and with SPADE_LIBRARY_WRITE
// set, their generator.
void expect_file_is_its_compiled_spec(std::string_view stem, std::string_view test) {
    const spade::Result<Committed> lib = committed_library();
    ASSERT_OK(lib);
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(lib->library, lib->registry, stem);
    ASSERT_OK(drone);
    const spade::Result<Generated> files = generate(*drone);
    ASSERT_OK(files);
    const std::array<std::pair<fs::path, const std::string*>, 2> outputs{{
        {library_dir() / fs::path(ld::model_file(stem)), &files->model},
        {library_dir() / fs::path(ld::sources_file(stem)), &files->sources},
    }};

    if (library_write_asked()) {
        for (const auto& [path, text] : outputs) {
            std::error_code ec;
            fs::create_directories(path.parent_path(), ec);
            ASSERT_FALSE(ec) << spade::path_text(path.parent_path()) << ": " << ec.message();
            {
                std::ofstream out(path, std::ios::binary | std::ios::trunc);
                out.write(text->data(), static_cast<std::streamsize>(text->size()));
                out.close();
                ASSERT_TRUE(out) << "SPADE_LIBRARY_WRITE: could not write " << spade::path_text(path);
            }
            std::string back;
            ASSERT_TRUE(read_bytes(path, back)) << spade::path_text(path);
            ASSERT_TRUE(back == *text) << "SPADE_LIBRARY_WRITE: " << spade::path_text(path)
                                       << " does not read back as written";
            std::printf("SPADE_LIBRARY_WRITE: wrote %zu bytes to %s.\n", text->size(), spade::path_text(path).c_str());
        }
        std::printf("SPADE_LIBRARY_WRITE: review the diff, name its cause (TD-1), and have a fresh Docker gcc leg "
                    "reproduce it (TD-12).\n");
        testing::Test::RecordProperty("written", spade::path_text(outputs[0].first));
        SUCCEED() << "wrote " << spade::path_text(outputs[0].first) << " and its sidecar";
        return;
    }

    for (const auto& [path, text] : outputs) {
        std::string committed;
        ASSERT_TRUE(read_bytes(path, committed))
            << spade::path_text(path) << " is missing (TD-5). Generate it: set SPADE_LIBRARY_WRITE=1 and run ctest -R "
            << "LibraryDrones." << test << ".";
        EXPECT_TRUE(committed == *text) << mismatch(path, committed, *text, test);
    }

    // The model loads back with its chain, and its parts are SDF parts, so no
    // .spmesh sidecar sits beside it.
    const spade::Result<ModelFile> loaded = sc::load_model_file(outputs[0].first);
    ASSERT_OK(loaded);
    EXPECT_TRUE(loaded->model.propulsion.has_value()) << "a library drone carries its chain";
    for (const spade::vehicles::PartDesc& part : loaded->model.parts) {
        EXPECT_TRUE(part.mesh == nullptr) << part.name << " is a mesh part: its sidecar must be committed beside it";
    }
}

// -- Every input cites a source (the draft's section 5) --------------------------

// Check 3's second half: the spec's blocks are the part files', field for
// field, but for what a part file does not carry (the ESC's and the pack's
// mount, the drone's) and comp's kappa-derived torque, which is checked to be
// kappa C_T / D exactly and to cite kappa.
void expect_blocks_are_the_part_files(const ld::BuiltDrone& drone, const Library& library,
                                      const SourceRegistry& registry) {
    for (std::size_t i = 0; i < ld::kPartKinds.size(); ++i) {
        const std::string& file = drone.parts.files[i];
        SCOPED_TRACE(file);
        const LibraryEntry* entry = entry_for(library, file);
        ASSERT_NE(entry, nullptr) << "library.yaml has no entry whose file is \"" << file << "\"";
        const spade::Result<PartFile> part = sc::load_library_part(library, *entry, registry);
        ASSERT_OK(part);
        const spade::Result<std::string> from_file = sc::part_to_yaml(*part);
        ASSERT_OK(from_file);
        std::optional<PartFile> from_spec;
        if (const auto* m = std::get_if<sc::MotorPart>(&*part)) {
            from_spec = PartFile{sc::MotorPart{drone.spec.motor, m->sources}};
        } else if (const auto* p = std::get_if<sc::PropPart>(&*part)) {
            spade::vehicles::PropBlock block = drone.spec.prop;
            if (drone.params && drone.params->kappa) {
                // C_Q = kappa C_T / D, the static point and the table at the
                // thrust table's J points (basic_quad_airframe(), THE FIT).
                const double kappa = *drone.params->kappa;
                const double d = p->block.diameter;
                EXPECT_EQ(bits(block.cq_static), bits(kappa * p->block.ct_static / d)) << "prop.cq_static";
                ASSERT_EQ(block.cq_table.size(), p->block.ct_table.size()) << "prop.cq_table";
                for (std::size_t j = 0; j < block.cq_table.size(); ++j) {
                    EXPECT_EQ(bits(block.cq_table[j][0]), bits(p->block.ct_table[j][0]))
                        << "prop.cq_table[" << j << "]";
                    EXPECT_EQ(bits(block.cq_table[j][1]), bits(kappa * p->block.ct_table[j][1] / d))
                        << "prop.cq_table[" << j << "]";
                }
                const ModelInput* cites = drone.inputs.find("kappa");
                ASSERT_NE(cites, nullptr) << "the kappa-derived torque cites kappa";
                EXPECT_NE(cites->ref.note.find("prop.cq_static"), std::string::npos) << cites->ref.note;
                EXPECT_NE(cites->ref.note.find("prop.cq_table"), std::string::npos) << cites->ref.note;
                block.cq_static = p->block.cq_static;
                block.cq_table = p->block.cq_table;
            }
            from_spec = PartFile{sc::PropPart{block, p->blade, p->sources}};
        } else if (const auto* e = std::get_if<sc::EscPart>(&*part)) {
            sc::EscPart esc{drone.spec.esc, e->sources};
            esc.block.mount = MountPose{};
            from_spec = PartFile{esc};
        } else if (const auto* b = std::get_if<sc::BatteryPart>(&*part)) {
            sc::BatteryPart pack{drone.spec.battery, b->sources};
            pack.block.mount = MountPose{};
            from_spec = PartFile{pack};
        }
        ASSERT_TRUE(from_spec.has_value());
        const spade::Result<std::string> spec_text = sc::part_to_yaml(*from_spec);
        EXPECT_OK(spec_text) << "the spec's block, emitted with the file's citations";
        if (spec_text) {
            EXPECT_EQ(*spec_text, *from_file) << "the spec's block is not the part file's, field for field";
        }
    }
}

// The part kind a basic_quad_assumptions() field belongs to, as an index into
// kPartKinds ("prop.ct_table" is 0), or none for the drone's own (a mount, the
// layout, the body).
[[nodiscard]] std::optional<std::size_t> block_of(std::string_view field) {
    if (field.starts_with("esc.mount") || field.starts_with("battery.mount")) return std::nullopt;
    for (std::size_t i = 0; i < ld::kPartKinds.size(); ++i) {
        const std::string prefix = std::string(sc::part_kind_name(ld::kPartKinds[i])) + ".";
        if (field.starts_with(prefix)) return i;
    }
    return std::nullopt;
}

// Checks 8 and 9, a basic quad's.
void expect_basic_quad_defaults(const ModelSources& sources, const Library& library, const SourceRegistry& registry) {
    ASSERT_EQ(sources.parts.size(), ld::kPartKinds.size());
    const BasicQuadParams defaults{};
    for (const spade::vehicles::BasicQuadAssumption& a : spade::vehicles::basic_quad_assumptions()) {
        const std::string field(a.field);
        SCOPED_TRACE(field);
        if (field == "rotors") {
            EXPECT_TRUE(contains(sources.assumptions, a.note)) << "the shared-wake line is in the assumptions";
            continue;
        }
        if (const std::optional<std::size_t> kind = block_of(field)) {
            // A block's field is cited by the part file the sidecar pins.
            const std::string& file = sources.parts[*kind].file;
            const LibraryEntry* entry = entry_for(library, file);
            ASSERT_NE(entry, nullptr) << file;
            const spade::Result<PartFile> part = sc::load_library_part(library, *entry, registry);
            ASSERT_OK(part);
            const std::string rest = field.substr(field.find('.') + 1);
            EXPECT_TRUE(contains(sc::cited_fields(*part), rest)) << file << " does not cite " << rest;
            continue;
        }
        std::size_t found = 0;
        for (const ModelInput& input : sources.inputs) {
            if (input.path != field && !input.path.starts_with(field + ".")) continue;
            ++found;
            const Found default_value = params_value(defaults, input.path);
            ASSERT_TRUE(default_value.has_value()) << input.path << " is not a BasicQuadParams field";
            // A default used unchanged: not one of Kat's values, and the default's own value.
            if (!input.kat && ld::same_value(input.value, *default_value)) {
                EXPECT_EQ(sc::grade_name(input.ref.grade), "assumed") << input.path << ": a default used unchanged";
                EXPECT_EQ(input.ref.note, a.note) << input.path << ": basic_quad_assumptions()' note, verbatim";
            }
        }
        EXPECT_GT(found, 0u) << field << ": every default basic_quad assumes appears among the inputs";
    }
    EXPECT_TRUE(contains(sources.assumptions, ld::kKappaRule)) << "section 4's kappa rule";
    EXPECT_TRUE(contains(sources.assumptions, ld::triblade_line())) << "the tri-blade stand-in's line";
}

void expect_every_input_cites_a_source(std::string_view stem) {
    const spade::Result<Committed> lib = committed_library();
    ASSERT_OK(lib);
    const Library& library = lib->library;
    const SourceRegistry& registry = lib->registry;
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(library, registry, stem);
    ASSERT_OK(drone);
    const std::string model = ld::model_file(stem);
    const LibraryEntry* entry = entry_for(library, model);
    ASSERT_NE(entry, nullptr) << "library.yaml has no entry whose file is \"" << model << "\"";
    EXPECT_EQ(entry->name, drone->def.index_name);
    EXPECT_EQ(entry->category, ld::kCategory);
    EXPECT_EQ(entry->kind, LibraryKind::model);

    // 1. The sidecar parses: the version gate, and no unknown, missing or
    // duplicate key.
    const spade::Result<ModelSources> sources =
        sc::load_model_sources(library_dir() / fs::path(ld::sources_file(stem)));
    ASSERT_OK(sources);

    // 2, 3 and 6: the model's bytes hash to model.hash; each part is an index
    // entry of its kind, hashes to its hash and loads; each citation is a
    // listed, usable source.
    EXPECT_OK(sc::check_model_sources(library, registry, *sources));
    EXPECT_EQ(sources->model.file, model);
    EXPECT_EQ(sources->built_by, drone->notes.built_by);

    // 3. The parts are the drone's, and the spec's blocks are the files'.
    ASSERT_EQ(sources->parts.size(), ld::kPartKinds.size());
    for (std::size_t i = 0; i < ld::kPartKinds.size(); ++i) {
        EXPECT_EQ(sources->parts[i].kind, ld::kPartKinds[i]) << "parts[" << i << "]";
        EXPECT_EQ(sources->parts[i].file, drone->parts.files[i]) << "parts[" << i << "]";
    }
    expect_blocks_are_the_part_files(*drone, library, registry);

    // 4. The inputs are the drone's table, path for path, in order.
    const std::vector<ModelInput>& table = drone->inputs.rows;
    ASSERT_EQ(sources->inputs.size(), table.size()) << "the sidecar's inputs are not the drone's table";
    for (std::size_t i = 0; i < table.size(); ++i) {
        const ModelInput& input = sources->inputs[i];
        const ModelInput& row = table[i];
        SCOPED_TRACE(row.path);
        EXPECT_EQ(input.path, row.path) << "inputs[" << i << "]";
        if (input.path != row.path) continue;

        // 5. Its value is the table's and the spec's or the params', bit for
        // bit, and its citation is the table's.
        EXPECT_TRUE(ld::same_value(input.value, row.value))
            << "the sidecar has " << text_of(input.value) << ", the table " << text_of(row.value);
        const Found built = drone_value(*drone, input.path);
        EXPECT_TRUE(built.has_value()) << input.path << " names no field of the drone's spec or params";
        if (built) {
            EXPECT_TRUE(ld::same_value(input.value, *built))
                << "the sidecar has " << text_of(input.value) << ", the drone " << text_of(*built);
        }
        EXPECT_EQ(input.ref.source, row.ref.source);
        EXPECT_EQ(sc::grade_name(input.ref.grade), sc::grade_name(row.ref.grade));
        EXPECT_EQ(input.ref.note, row.ref.note);
        EXPECT_EQ(input.kat.has_value(), row.kat.has_value());
        if (input.kat && row.kat) {
            EXPECT_EQ(input.kat->source, row.kat->source);
            EXPECT_EQ(input.kat->confidence, row.kat->confidence);
            EXPECT_EQ(input.kat->date, row.kat->date);
        }

        // 6. By name.
        EXPECT_OK(sc::check_source_ref(registry, input.ref, "inputs." + input.path));

        // 7. No input cites the withdrawn datapack.
        EXPECT_FALSE(is_withdrawn(input.ref.source))
            << input.path << " cites " << input.ref.source << ", whose section 7.2 fit was withdrawn";
    }

    // The checks, what is not carried, the assumptions and the caveat are the
    // drone's; 6 and 7 for each check.
    ASSERT_EQ(sources->checks.size(), drone->notes.checks.size());
    for (std::size_t i = 0; i < sources->checks.size(); ++i) {
        const ModelCheck& c = sources->checks[i];
        const ModelCheck& want = drone->notes.checks[i];
        SCOPED_TRACE("checks[" + std::to_string(i) + "] " + c.test);
        EXPECT_EQ(c.test, want.test);
        EXPECT_EQ(bits(c.reference), bits(want.reference));
        EXPECT_EQ(c.unit, want.unit);
        EXPECT_EQ(c.ref.source, want.ref.source);
        EXPECT_EQ(sc::grade_name(c.ref.grade), sc::grade_name(want.ref.grade));
        EXPECT_EQ(c.ref.note, want.ref.note);
        EXPECT_EQ(c.band.has_value(), want.band.has_value());
        if (c.band && want.band) {
            EXPECT_EQ(bits((*c.band)[0]), bits((*want.band)[0])) << "band[0]";
            EXPECT_EQ(bits((*c.band)[1]), bits((*want.band)[1])) << "band[1]";
        }
        EXPECT_EQ(c.report_only, want.report_only);
        EXPECT_OK(sc::check_source_ref(registry, c.ref, "checks[" + std::to_string(i) + "]"));
        if (c.ref.grade == Grade::fitted) {
            EXPECT_FALSE(is_withdrawn(c.ref.source)) << "no fitted grade cites withdrawn work";
        }
    }
    EXPECT_EQ(sources->not_carried, drone->notes.not_carried);
    EXPECT_EQ(sources->assumptions, drone->notes.assumptions);
    EXPECT_EQ(sources->caveat, drone->notes.caveat);

    // 8 and 9.
    if (drone->params) {
        expect_basic_quad_defaults(*sources, library, registry);
    }
}

// -- The checks ------------------------------------------------------------------

// The rpm at which the committed model's four k_T carry NASA's 12.46 N, held to
// its check's band (Physics' +-5%, like for like).
void expect_hovers_at_nasas_baseline_rpm(std::string_view stem, std::string_view test, double hand_rpm) {
    const spade::Result<Committed> lib = committed_library();
    ASSERT_OK(lib);
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(lib->library, lib->registry, stem);
    ASSERT_OK(drone);
    const spade::Result<ModelFile> file = drone_model(lib->library, stem);
    ASSERT_OK(file);
    const ModelCheck* check = banded_check(drone->notes, test);
    ASSERT_NE(check, nullptr) << stem << " has no banded check " << test;
    ASSERT_EQ(file->model.rotors.size(), 4u);

    const double rpm = rpm_of(std::sqrt(ld::kNasaHoverThrust / thrust_coeff_sum(file->model)));
    const double relative = (rpm - check->reference) / check->reference;
    std::printf("[ check    ] %.*s: %.1f rpm at NASA's %.2f N against %.0f: %+.2f%% (hand figure %.0f)\n",
                static_cast<int>(stem.size()), stem.data(), rpm, ld::kNasaHoverThrust, check->reference,
                100.0 * relative, hand_rpm);
    testing::Test::RecordProperty("rpm_at_nasa_thrust", std::to_string(rpm));
    expect_in_band(*check, relative, "the rpm at NASA's 12.46 N");

    // Report only: the drone at its own mass.
    report_hover_and_full_duty(*drone, *file, std::string(stem));
}

}  // namespace

// ===========================================================================
// The goldens, and their generator
// ===========================================================================

TEST(LibraryDrones, Phantom3FileIsItsCompiledSpec) {
    expect_file_is_its_compiled_spec("phantom3_class", "Phantom3FileIsItsCompiledSpec");
}

TEST(LibraryDrones, IrisPlusFileIsItsCompiledSpec) {
    expect_file_is_its_compiled_spec("iris_plus_class", "IrisPlusFileIsItsCompiledSpec");
}

TEST(LibraryDrones, Fpv5InchFileIsItsCompiledSpec) {
    expect_file_is_its_compiled_spec("fpv_5inch", "Fpv5InchFileIsItsCompiledSpec");
}

TEST(LibraryDrones, CrazyflieFileIsItsCompiledSpec) {
    expect_file_is_its_compiled_spec("crazyflie_class", "CrazyflieFileIsItsCompiledSpec");
}

TEST(LibraryDrones, BasicQuad6InchFileIsItsCompiledSpec) {
    expect_file_is_its_compiled_spec("basic_quad_6inch", "BasicQuad6InchFileIsItsCompiledSpec");
}

TEST(LibraryDrones, BasicQuadCompetitionFileIsItsCompiledSpec) {
    expect_file_is_its_compiled_spec("basic_quad_competition", "BasicQuadCompetitionFileIsItsCompiledSpec");
}

// ===========================================================================
// Every input cites a source
// ===========================================================================

TEST(LibraryDrones, Phantom3EveryInputCitesASource) { expect_every_input_cites_a_source("phantom3_class"); }

TEST(LibraryDrones, IrisPlusEveryInputCitesASource) { expect_every_input_cites_a_source("iris_plus_class"); }

TEST(LibraryDrones, Fpv5InchEveryInputCitesASource) { expect_every_input_cites_a_source("fpv_5inch"); }

TEST(LibraryDrones, CrazyflieEveryInputCitesASource) { expect_every_input_cites_a_source("crazyflie_class"); }

TEST(LibraryDrones, BasicQuad6InchEveryInputCitesASource) { expect_every_input_cites_a_source("basic_quad_6inch"); }

TEST(LibraryDrones, BasicQuadCompetitionEveryInputCitesASource) {
    expect_every_input_cites_a_source("basic_quad_competition");
}

// ===========================================================================
// The checks (Physics, "L5's checks", 2026-10-22)
// ===========================================================================

// By hand (the L5 draft, 3.1): 5405 rpm, +1.98% against 5300. The curve is
// UIUC's APC TE 10x5 at 9.4 in, not DJI's 9450.
TEST(LibraryDrones, Phantom3HoversAtNasasBaselineRpm) {
    expect_hovers_at_nasas_baseline_rpm("phantom3_class", "Phantom3HoversAtNasasBaselineRpm", 5405.0);
}

// By hand (the L5 draft, 3.2): 5137 rpm, -4.87% against 5400, 0.13 pp inside
// the band. That is explained, not a defect (Physics, 2026-10-22): the 9x4.5's
// static C_T sits about 10% above its run's low-J points (UIUC's tunnel at its
// slowest), and sqrt(1.10) is about 4.9% in rpm, so the static point the
// model hovers on reads that much low. It is checked at NASA's 12.46 N, like
// for like; its own mass (about -4.4%) is report-only.
TEST(LibraryDrones, IrisPlusHoversAtNasasBaselineRpm) {
    expect_hovers_at_nasas_baseline_rpm("iris_plus_class", "IrisPlusHoversAtNasasBaselineRpm", 5137.0);
}

// One motor's k_T w^2 / g at Bitcraze's 17,174 rpm row against a quarter of the
// whole craft's 28.6 g: by hand 7.78 g against 7.15 g, +8.8%, inside the
// UNWIDENED +-10%. UIUC measured an isolated prop and Bitcraze weighed the
// whole craft (Physics, 2026-10-22). Report only: the hover rpm against the
// table's 17,270; the two loaded rows, each at its own duty and at the state
// of charge whose loaded bus reads its voltage (the top row is R's own fit
// point, so it closes the loop, not an independent check); and full duty at
// SoC 1 (full pack).
TEST(LibraryDrones, CrazyflieMatchesBitcrazesThrustTable) {
    const spade::Result<Committed> lib = committed_library();
    ASSERT_OK(lib);
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(lib->library, lib->registry, "crazyflie_class");
    ASSERT_OK(drone);
    const spade::Result<ModelFile> file = drone_model(lib->library, "crazyflie_class");
    ASSERT_OK(file);
    const ModelCheck* check = banded_check(drone->notes, "CrazyflieMatchesBitcrazesThrustTable");
    ASSERT_NE(check, nullptr);
    const spade::vehicles::ModelType& m = file->model;
    ASSERT_EQ(m.rotors.size(), 4u);
    ASSERT_TRUE(m.propulsion.has_value());

    const double w = omega_of(ld::kCfRowRpm);
    const double grams = static_cast<double>(m.rotors[0].thrust_coeff) * w * w / drone->spec.gravity * 1000.0;
    const double relative = (grams - check->reference) / check->reference;
    std::printf("[ check    ] crazyflie_class: %.3f g per motor at %.0f rpm against %.3f g: %+.2f%% (hand figure "
                "7.78 g, +8.8%%)\n",
                grams, ld::kCfRowRpm, check->reference, 100.0 * relative);
    testing::Test::RecordProperty("grams_per_motor_at_row", std::to_string(grams));
    expect_in_band(*check, relative, "one motor's thrust at Bitcraze's 17,174 rpm row");

    // Report only.
    const AirframeSpec& s = drone->spec;
    const double weight = static_cast<double>(m.body.mass) * s.gravity;
    const double hover_rpm = rpm_of(std::sqrt(weight / thrust_coeff_sum(m)));
    report("crazyflie_hover_rpm", hover_rpm, "own mass; Bitcraze's table gives about 17,270 (hand -4.0%)");
    report("crazyflie_hover_rpm_vs_table", (hover_rpm - ld::kCfTableHoverRpm) / ld::kCfTableHoverRpm, "relative");
    const std::vector<const ModelCheck*> reported = report_checks(drone->notes, "CrazyflieMatchesBitcrazesThrustTable");
    EXPECT_EQ(reported.size(), 3u) << "the two loaded rows and full duty at SoC 1, all report-only";
    const spade::vehicles::PropulsionChain& chain = *m.propulsion;
    const double motors = static_cast<double>(chain.motors_on_bus);
    struct LoadedRow {
        std::string key;
        double duty;
        double bus_voltage;
        double rpm;
        double craft_current;
    };
    const std::array<LoadedRow, 2> rows{{
        {"crazyflie_top_row", ld::kCfTopRowDuty, ld::kCfTopRowBusVoltage, ld::kCfTopRowRpm, ld::kCfTopRowCraftCurrent},
        {"crazyflie_17174_row", ld::kCfRowDuty, ld::kCfRowBusVoltage, ld::kCfRowRpm, ld::kCfRowCraftCurrent},
    }};
    for (const LoadedRow& row : rows) {
        const double soc = soc_at_loaded_bus(chain, row.duty, s.air_density, row.bus_voltage);
        const SteadyPoint p = spade::vehicles::steady_state_at_duty(chain, row.duty, s.air_density, 0.0, soc);
        const double rpm = rpm_of(p.omega);
        report(row.key + "_soc", soc, "the state of charge whose loaded bus reads the row's voltage");
        report(row.key + "_bus_voltage", p.bus_voltage, "V, loaded, against the row's");
        report(row.key + "_rpm", rpm, "at the row's duty");
        report(row.key + "_rpm_vs_row", (rpm - row.rpm) / row.rpm, "relative, against the row's rpm");
        report(row.key + "_bus_current_per_motor", p.pack_current / motors, "A, the pack's over its motors");
        report(row.key + "_row_bus_current_per_motor", (row.craft_current - ld::kCfBoardCurrent) / 4.0,
               "A, the row's (I - 0.24 A for the board) / 4");
        report(row.key + "_winding_current", p.current, "A per motor");
        report(row.key + "_flags", static_cast<double>(p.flags), "propulsion_flags");
    }
    const SteadyPoint full = spade::vehicles::steady_state_at_duty(*m.propulsion, 1.0, s.air_density, 0.0, 1.0);
    report("crazyflie_full_rpm_at_soc1", rpm_of(full.omega), "full pack; Bitcraze's top row 23,882 (hand 24,757)");
    report("crazyflie_tau", static_cast<double>(m.rotors[0].tau), "s (hand 63 ms)");
}

// Physics' precondition for the refit (2026-10-22): the chain's pack current is
// the sum over its motors of duty x winding current, so the fit's bus / duty
// reading and the chain agree on which current is which. A chain summing the
// winding currents without the duty fails both expectations: below full duty
// the pack carries less than the windings (6.25% less at the top row's duty,
// 43.75% at the 17,174 rpm row's).
TEST(LibraryDrones, CrazyfliesPackCurrentIsTheSumOfDutyTimesWindingCurrent) {
    const spade::Result<Committed> lib = committed_library();
    ASSERT_OK(lib);
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(lib->library, lib->registry, "crazyflie_class");
    ASSERT_OK(drone);
    const spade::Result<ModelFile> file = drone_model(lib->library, "crazyflie_class");
    ASSERT_OK(file);
    ASSERT_TRUE(file->model.propulsion.has_value());
    const spade::vehicles::PropulsionChain& chain = *file->model.propulsion;
    ASSERT_EQ(chain.motors_on_bus, 4u);
    const double density = drone->spec.air_density;
    for (const auto& [duty, volts] : {std::pair{ld::kCfRowDuty, ld::kCfRowBusVoltage},
                                      std::pair{ld::kCfTopRowDuty, ld::kCfTopRowBusVoltage}}) {
        const double soc = soc_at_loaded_bus(chain, duty, density, volts);
        const SteadyPoint p = spade::vehicles::steady_state_at_duty(chain, duty, density, 0.0, soc);
        const double sum = 4.0 * p.duty * p.duty_scale * p.current;
        EXPECT_NEAR(p.pack_current, sum, 1e-12 * sum) << "duty " << duty << ", flags " << p.flags;
        EXPECT_LT(p.pack_current, 4.0 * p.current)
            << "duty " << duty << ": below full duty the pack carries less than the windings";
    }
}

// REPORT-ONLY (Physics, 2026-10-22): 4 x the full-duty thrust over m g, which
// typical 5-inch racers put around 8-14, with no single source, so no band is
// held. By hand 10.16, held by the pack (97.5 A); the motors draw 33.2 A,
// under their 35.08 A. Only the chain's solving it is expected.
TEST(LibraryDrones, FpvThrustToWeightIsReported) {
    const spade::Result<Committed> lib = committed_library();
    ASSERT_OK(lib);
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(lib->library, lib->registry, "fpv_5inch");
    ASSERT_OK(drone);
    const spade::Result<ModelFile> file = drone_model(lib->library, "fpv_5inch");
    ASSERT_OK(file);
    const spade::vehicles::ModelType& m = file->model;
    ASSERT_TRUE(m.propulsion.has_value());
    ASSERT_EQ(report_checks(drone->notes, "FpvThrustToWeightIsReported").size(), 1u);

    const AirframeSpec& s = drone->spec;
    const SteadyPoint full = spade::vehicles::steady_state_at_duty(*m.propulsion, 1.0, s.air_density, 0.0,
                                                                   s.state_of_charge);
    EXPECT_EQ(full.flags & (pf::no_bracket | pf::unreachable), 0u) << "the chain solves at full duty";
    const double weight = static_cast<double>(m.body.mass) * s.gravity;
    report("fpv_thrust_to_weight", 4.0 * full.thrust / weight,
           "typical 5-inch racers sit around 8-14 (no single source); hand 10.16");
    report("fpv_full_pack_current", full.pack_current, "A (the pack's 97.5 A)");
    report("fpv_full_motor_current", full.current, "A per motor (hand 33.2, under 35.08)");
    report("fpv_full_flags", static_cast<double>(full.flags), "propulsion_flags at full duty");
    report_hover_and_full_duty(*drone, *file, "fpv");
}

// The 6-inch defaults hover: the committed chain's fit converges (a duty in
// (0, 1), no limit flag), and its hover rpm is the L4a header's hover_rpm
// within +-0.1%. By hand 6658.7 against 6656.3, +0.04%: the header used
// 0.602 kg, the parts sum to 0.6024 kg. A larger gap means the header's run
// choice must be re-checked.
TEST(LibraryDrones, BasicQuadDefaultsHover) {
    const spade::Result<Committed> lib = committed_library();
    ASSERT_OK(lib);
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(lib->library, lib->registry, "basic_quad_6inch");
    ASSERT_OK(drone);
    const spade::Result<ModelFile> file = drone_model(lib->library, "basic_quad_6inch");
    ASSERT_OK(file);
    const ModelCheck* check = banded_check(drone->notes, "BasicQuadDefaultsHover");
    ASSERT_NE(check, nullptr);
    const spade::vehicles::ModelType& m = file->model;
    ASSERT_TRUE(m.propulsion.has_value());
    ASSERT_EQ(m.rotors.size(), 4u);

    const AirframeSpec& s = drone->spec;
    const double weight = static_cast<double>(m.body.mass) * s.gravity;
    const SteadyPoint hover =
        spade::vehicles::steady_state_for_thrust(*m.propulsion, weight / 4.0, s.air_density, 0.0, s.state_of_charge);
    EXPECT_GT(hover.duty, 0.0);
    EXPECT_LT(hover.duty, 1.0);
    EXPECT_EQ(hover.flags & kLimitFlags, 0u) << "propulsion_flags 0x" << std::hex << hover.flags;
    const double rpm = rpm_of(hover.omega);
    const double relative = (rpm - check->reference) / check->reference;
    std::printf("[ check    ] basic_quad_6inch: hovers at %.1f rpm against the header's %.1f: %+.3f%% (hand figure "
                "6658.7, +0.04%%)\n",
                rpm, check->reference, 100.0 * relative);
    testing::Test::RecordProperty("hover_rpm", std::to_string(rpm));
    expect_in_band(*check, relative, "the hover rpm against the L4a header's");

    report("basic6_hover_duty", hover.duty, "hand 0.169");
    report("basic6_hover_current", hover.current, "A per motor (hand 5.18)");
    report_hover_and_full_duty(*drone, *file, "basic6");
}

// A GUARD, named as such (Physics, 2026-10-22): that the mass input is
// consistent with Kat's curve. The compiled hover thrust per rotor (m g / 4)
// against the curve's hover point (0.700 kg at 9.712 m/s^2, / 4), within
// +-2%; by hand 1.71616 N against 1.69960 N, +0.97%, the curve's own implied
// g. The curve (kat_competition_700, measured 2026-07-29) was never
// withdrawn, so this stays banded. It tests the fitted mass, not the parts.
// REPORT-ONLY, because the parts meant to meet the curve came from the
// withdrawn datapack 7.2: full duty gives about 6.75 N per motor against
// Kat's 9.188 N (-26.5%), current-limited at 40 A on kappa's torque, where the
// table's C_Q would give 9.34 N. Duty against norm 0.264 is dropped (norm is a
// thrust command, not a duty).
TEST(LibraryDrones, BasicQuadCompetitionHoversAtTheCurvesHoverPoint) {
    const spade::Result<Committed> lib = committed_library();
    ASSERT_OK(lib);
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(lib->library, lib->registry, "basic_quad_competition");
    ASSERT_OK(drone);
    const spade::Result<ModelFile> file = drone_model(lib->library, "basic_quad_competition");
    ASSERT_OK(file);
    const ModelCheck* check = banded_check(drone->notes, "BasicQuadCompetitionHoversAtTheCurvesHoverPoint");
    ASSERT_NE(check, nullptr);
    const ModelInput* kat_mass = drone->inputs.find("mass");
    ASSERT_NE(kat_mass, nullptr);
    ASSERT_TRUE(std::holds_alternative<double>(kat_mass->value));
    const spade::vehicles::ModelType& m = file->model;
    ASSERT_TRUE(m.propulsion.has_value());
    ASSERT_EQ(m.rotors.size(), 4u);

    const AirframeSpec& s = drone->spec;
    const double per_rotor = static_cast<double>(m.body.mass) * s.gravity / 4.0;
    const double curve = std::get<double>(kat_mass->value) * check->reference / 4.0;
    const double relative = (per_rotor - curve) / curve;
    std::printf("[ guard    ] basic_quad_competition: hover thrust %.5f N per rotor against the curve's %.5f N: "
                "%+.2f%% (hand figure +0.97%%)\n",
                per_rotor, curve, 100.0 * relative);
    testing::Test::RecordProperty("hover_thrust_per_rotor", std::to_string(per_rotor));
    expect_in_band(*check, relative, "the guard: the compiled hover thrust against the curve's hover point");

    // Report only: kappa's cost, with the table's C_Q beside it.
    const std::vector<const ModelCheck*> reported =
        report_checks(drone->notes, "BasicQuadCompetitionHoversAtTheCurvesHoverPoint");
    ASSERT_EQ(reported.size(), 1u);
    const SteadyPoint full =
        spade::vehicles::steady_state_at_duty(*m.propulsion, 1.0, s.air_density, 0.0, s.state_of_charge);
    report("comp_full_thrust", full.thrust, "N per motor; Kat's 9.188 (hand 6.75, -26.5%)");
    report("comp_full_vs_kat", (full.thrust - reported[0]->reference) / reported[0]->reference, "relative");
    report("comp_full_flags", static_cast<double>(full.flags), "current_limited expected, at 40 A");
    ASSERT_TRUE(drone->params.has_value());
    BasicQuadParams table_cq = *drone->params;
    table_cq.kappa.reset();
    const spade::Result<AirframeSpec> untorqued = spade::vehicles::basic_quad_airframe(table_cq);
    ASSERT_OK(untorqued);
    const auto chain = spade::vehicles::airframe_propulsion_chain(*untorqued);
    ASSERT_OK(chain);
    const SteadyPoint table = spade::vehicles::steady_state_at_duty(*chain, 1.0, s.air_density, 0.0, s.state_of_charge);
    report("comp_full_thrust_on_table_cq", table.thrust, "N per motor, the table's C_Q (hand 9.34)");
    const SteadyPoint hover =
        spade::vehicles::steady_state_for_thrust(*m.propulsion, per_rotor, s.air_density, 0.0, s.state_of_charge);
    report("comp_hover_duty", hover.duty, "hand 0.210; norm 0.264 is a thrust command, not compared");

    // L3's margin line for the 2207/4S parts: the remainder's smaller two
    // moments less its largest (its off-diagonals are rounding).
    ASSERT_EQ(s.parts.size(), 6u);
    const PartInertia& rest = s.parts[5];
    std::array<double, 3> moments{rest.tensor[0][0], rest.tensor[1][1], rest.tensor[2][2]};
    std::sort(moments.begin(), moments.end());
    report("comp_remainder_margin", moments[0] + moments[1] - moments[2], "kg m^2, I1 + I2 - I3 (hand 2.12e-5)");
    report("comp_remainder_mass", rest.mass, "kg (hand 0.1414)");
}

// Every kat_competition_700 input carries Kat's three fields, equal to the
// JSON's, and the sidecar states the caveat (Review Focus 6). Kat's values as
// read 2026-10-21 from content/airframes/competition-700.airframe.json, its
// provenance by block, and its values in Spade's design axes: the arms are
// arm_x and arm_y; the inertia is its FLU (Ixx, Iyy, Izz) = (0.0025, 0.0021,
// 0.0043) as (x roll, y yaw, z pitch); its inertia is "undated", so the
// sidecar leaves the date out.
TEST(LibraryDrones, BasicQuadCompetitionCarriesKatsProvenance) {
    struct KatValue {
        std::string_view path;
        std::string_view block;  // the JSON's provenance block
        std::string_view source;
        std::string_view confidence;
        std::string_view date;  // empty: "undated"
        std::vector<double> value;
        Grade grade;  // as ruled (the plan; the lead and Physics, 2026-10-22)
    };
    const std::vector<KatValue> kat{
        {"half_span_x", "motor", "derived", "medium", "2026-07-29", {0.075}, Grade::assumed},
        {"half_span_z", "motor", "derived", "medium", "2026-07-29", {0.1}, Grade::assumed},
        {"mass", "mass", "tuned", "medium", "2026-07-27", {0.7}, Grade::fitted},
        {"inertia", "inertia", "assumed", "low", "", {0.0025, 0.0043, 0.0021}, Grade::assumed},
        {"kappa", "motor", "derived", "medium", "2026-07-29", {0.022}, Grade::assumed},
        {"drag[0].coeffs", "drag", "tuned", "high", "2026-07-29", {0.028, 0.028, 0.028}, Grade::fitted},
    };
    const spade::Result<ModelSources> sources =
        sc::load_model_sources(library_dir() / fs::path(ld::sources_file("basic_quad_competition")));
    ASSERT_OK(sources);

    for (const KatValue& k : kat) {
        SCOPED_TRACE(std::string(k.path) + " (Kat's " + std::string(k.block) + ")");
        const ModelInput* input = nullptr;
        for (const ModelInput& i : sources->inputs) {
            if (i.path == k.path) input = &i;
        }
        ASSERT_NE(input, nullptr) << "the sidecar has no input " << k.path;
        EXPECT_EQ(input->ref.source, ld::kKat700);
        EXPECT_EQ(sc::grade_name(input->ref.grade), sc::grade_name(k.grade));
        const InputValue want = k.value.size() == 1 ? InputValue{k.value[0]} : InputValue{k.value};
        EXPECT_TRUE(ld::same_value(input->value, want)) << text_of(input->value) << ", Kat's " << text_of(want);
        ASSERT_TRUE(input->kat.has_value()) << "Kat's provenance";
        EXPECT_EQ(input->kat->source, k.source);
        EXPECT_EQ(input->kat->confidence, k.confidence);
        EXPECT_EQ(input->kat->date, k.date);
    }

    // Every input citing Kat's file is one of Kat's values above, with Kat's
    // three fields.
    for (const ModelInput& input : sources->inputs) {
        if (input.ref.source != ld::kKat700) continue;
        SCOPED_TRACE(input.path);
        const bool listed =
            std::any_of(kat.begin(), kat.end(), [&](const KatValue& k) { return k.path == input.path; });
        EXPECT_TRUE(listed) << input.path << " cites kat_competition_700 but is not one of Kat's values";
        EXPECT_TRUE(input.kat.has_value()) << "every kat_competition_700 input carries Kat's provenance";
        if (input.kat) {
            EXPECT_FALSE(input.kat->source.empty());
            EXPECT_FALSE(input.kat->confidence.empty());
        }
    }

    // The caveat: Kat's simulator, and kappa's cost (Physics, 2026-10-22).
    EXPECT_NE(sources->caveat.find(ld::kCompSimulator), std::string::npos) << sources->caveat;
    EXPECT_NE(sources->caveat.find(ld::kCompKappaCost), std::string::npos) << sources->caveat;

    // What the model does not carry, by Kat's key: the curve and its airspeed
    // loss (ruling 6), the controller's values, and the collision radius
    // (Physics: a planner's margin; proxy_radius is the geometric reach).
    for (const std::string_view key :
         {"thrust_curve", "airspeed_thrust", "motor.tau", "envelope", "rate_loop", "collision_radius"}) {
        const std::string prefix = std::string(key) + ":";
        const bool named = std::any_of(sources->not_carried.begin(), sources->not_carried.end(),
                                       [&](const std::string& n) { return n.starts_with(prefix); });
        EXPECT_TRUE(named) << "not_carried names " << key;
    }
}

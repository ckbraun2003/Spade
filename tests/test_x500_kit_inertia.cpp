// ---------------------------------------------------------------------------
// test_x500_kit_inertia.cpp -- the X500 kit's mass, centre of mass and inertia from the build table's parts (Core's tools/flow_movie/data/x500_build/
// parts.yaml), each part at the table's pose: a part with store meshes (the frame's 61 objects, the motors, the controller, the GPS, the Pi, the camera)
// from the meshes' own polyhedral mass properties (TriangleMesh::mass_properties(): volume, centroid, inertia per unit density, exact for the shape),
// scaled to the part's measured mass at uniform density; a part the table gives as a box (the pack, the radio, the misc carrier) as a uniform box; a
// rotor as a thin disc of its radius. It prints the composite's centre of mass (against the table's own), the full tensor about it, its principal
// moments, and how far the principal axes turn from the design axes (the engine's model is principal-axes only, L88l).
//
// OPT-IN: SPADE_X500_KIT_INERTIA=1, and SPADE_PART_STORE the shared part store folder (<store>/<sha256>/<file>.spmesh; the CAD stays local, L88b).
// Without them the test skips: the meshes are not in git.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <yaml-cpp/yaml.h>

#include "world/mesh.hpp"
#include "world/mesh_file.hpp"

namespace {

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
const char* env_text(const char* name) {
    return std::getenv(name);
}
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

const std::string kTable = std::string(SPADE_REPO_DIR) + "/tools/flow_movie/data/x500_build/parts.yaml";

struct Body {
    double mass = 0.0;
    glm::dvec3 centre{0.0};  // the table's frame
    glm::dmat3 tensor{0.0};  // about `centre`, the table's axes
};

glm::dvec3 vec3_of(const YAML::Node& n) {
    return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}

glm::dmat3 rotation_of(const YAML::Node& n) {
    return glm::mat3_cast(glm::dquat(n[0].as<double>(), n[1].as<double>(), n[2].as<double>(), n[3].as<double>()));
}

glm::dmat3 box_tensor(double mass, const glm::dvec3& d) {
    glm::dmat3 t(0.0);
    t[0][0] = mass * (d.y * d.y + d.z * d.z) / 12.0;
    t[1][1] = mass * (d.x * d.x + d.z * d.z) / 12.0;
    t[2][2] = mass * (d.x * d.x + d.y * d.y) / 12.0;
    return t;
}

// The inertia of a point mass `m` at `r` from the reference point, as a tensor.
glm::dmat3 parallel_axis(double m, const glm::dvec3& r) {
    glm::dmat3 t(0.0);
    const double r2 = glm::dot(r, r);
    for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 3; ++k) t[c][k] = m * ((c == k ? r2 : 0.0) - r[c] * r[k]);
    }
    return t;
}

// A symmetric 3x3's eigenvalues and eigenvectors (columns of `v`), by cyclic Jacobi sweeps.
void jacobi(glm::dmat3 a, std::array<double, 3>& values, glm::dmat3& v) {
    v = glm::dmat3(1.0);
    for (int sweep = 0; sweep < 60; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < 3; ++p) {
            for (int q = p + 1; q < 3; ++q) off += a[q][p] * a[q][p];
        }
        if (off < 1e-30) break;
        for (int p = 0; p < 3; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (std::abs(a[q][p]) < 1e-300) continue;
                const double theta = (a[q][q] - a[p][p]) / (2.0 * a[q][p]);
                const double t = (theta >= 0.0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double s = t * c;
                glm::dmat3 rot(1.0);
                rot[p][p] = c;
                rot[q][q] = c;
                rot[q][p] = s;
                rot[p][q] = -s;
                a = glm::transpose(rot) * a * rot;
                v = v * rot;
            }
        }
    }
    for (int i = 0; i < 3; ++i) values[static_cast<std::size_t>(i)] = a[i][i];
}

TEST(X500KitInertia, TheKitsMassCentreAndInertiaFromItsPartsMeshes) {
    const char* enable = env_text("SPADE_X500_KIT_INERTIA");
    const char* store_text = env_text("SPADE_PART_STORE");
    if (enable == nullptr || store_text == nullptr) {
        GTEST_SKIP() << "set SPADE_X500_KIT_INERTIA=1 and SPADE_PART_STORE=<the shared part store> to run the kit's inertia from its meshes";
    }
    const std::filesystem::path store(store_text);
    const YAML::Node doc = YAML::LoadFile(kTable);
    std::vector<Body> bodies;
    std::vector<std::string> names;
    for (const YAML::Node& part : doc["parts"]) {
        const std::string name = part["name"].as<std::string>();
        const std::string kind = part["kind"].as<std::string>();
        const double mass = part["mass_kg"].as<double>();
        const YAML::Node pose = part["pose"];
        const glm::dvec3 at = vec3_of(pose["position"]);
        const glm::dmat3 rot = rotation_of(pose["orientation"]);
        std::string basis;
        if (kind != "rotor" && part["objects"]) {
            // The part's meshes: each object's volume weights the part's mass (uniform density); a mesh's centroid and tensor go through the pose.
            struct Piece {
                double volume;
                glm::dvec3 centroid;
                glm::dmat3 inertia;
            };
            std::vector<Piece> pieces;
            double volume = 0.0;
            for (const YAML::Node& object : part["objects"]) {
                const std::filesystem::path file =
                    store / object["store_ref"]["sha256"].as<std::string>() / object["store_ref"]["file"].as<std::string>();
                const auto loaded = spade::load_mesh_file(file);
                ASSERT_TRUE(loaded.has_value()) << file.string() << ": " << loaded.error().context;
                const spade::MeshMassProperties& m = loaded->mesh->mass_properties();
                pieces.push_back({m.volume, at + rot * m.centroid, rot * m.inertia * glm::transpose(rot)});
                const std::string stem = object["store_ref"]["file"].as<std::string>();
                if (stem.find("TUBE") != std::string::npos || stem.find("PLATE") != std::string::npos) {
                    const glm::dvec3 c = at + rot * m.centroid;
                    std::printf("[kit-object] %-34s volume %.4e m^3 centroid (%.4f, %.4f, %.4f)\n", stem.c_str(), m.volume, c.x, c.y, c.z);
                }
                volume += m.volume;
            }
            Body b;
            b.mass = mass;
            for (const Piece& p : pieces) b.centre += (mass * p.volume / volume) * p.centroid;
            b.centre /= mass;
            for (const Piece& p : pieces) {
                const double mi = mass * p.volume / volume;
                b.tensor += (mi / p.volume) * p.inertia + parallel_axis(mi, p.centroid - b.centre);
            }
            bodies.push_back(b);
            basis = "meshes (" + std::to_string(pieces.size()) + ")";
        } else if (kind != "rotor" && part["box"]) {
            Body b;
            b.mass = mass;
            b.centre = at;
            b.tensor = rot * box_tensor(mass, vec3_of(part["box"])) * glm::transpose(rot);
            bodies.push_back(b);
            basis = "box";
        } else if (kind == "rotor") {
            // A thin disc of the prop's radius about its spin axis (y): 1/2 m R^2 about it, 1/4 m R^2 across; the prop's blades and hub are thin.
            const double radius = 0.127;
            Body b;
            b.mass = mass;
            b.centre = at;
            b.tensor[0][0] = 0.25 * mass * radius * radius;
            b.tensor[1][1] = 0.5 * mass * radius * radius;
            b.tensor[2][2] = 0.25 * mass * radius * radius;
            bodies.push_back(b);
            basis = "thin disc R 0.127";
        } else {
            ADD_FAILURE() << name << " (" << kind << ") has neither meshes, a box nor a rotor";
            continue;
        }
        names.push_back(name);
        std::printf("[kit] %-22s %-10s %.4f kg  %s  centre (%.4f, %.4f, %.4f)\n", name.c_str(), kind.c_str(), mass, basis.c_str(),
                    bodies.back().centre.x, bodies.back().centre.y, bodies.back().centre.z);
    }
    double mass = 0.0;
    glm::dvec3 centre(0.0);
    for (const Body& b : bodies) {
        mass += b.mass;
        centre += b.mass * b.centre;
    }
    centre /= mass;
    glm::dmat3 tensor(0.0);
    for (const Body& b : bodies) tensor += b.tensor + parallel_axis(b.mass, b.centre - centre);

    std::printf("[kit] total %.4f kg (the table's sum 1.6326); centre of mass (%.5f, %.5f, %.5f) (the table's -0.00448, -0.02581, 0.00056)\n", mass,
                centre.x, centre.y, centre.z);
    std::printf("[kit] tensor about the centre of mass, kg m^2 (rows x, y, z):\n");
    for (int r = 0; r < 3; ++r) std::printf("[kit]   %+.6f %+.6f %+.6f\n", tensor[0][r], tensor[1][r], tensor[2][r]);
    std::array<double, 3> values{};
    glm::dmat3 v(1.0);
    jacobi(tensor, values, v);
    // Pair each design axis with the eigenvector nearest it, flip signs to a proper rotation, and read the turn.
    glm::dmat3 pairing(0.0);
    std::array<double, 3> principal{};
    std::array<bool, 3> used{false, false, false};
    for (int axis = 0; axis < 3; ++axis) {
        int best = -1;
        double best_dot = -1.0;
        for (int e = 0; e < 3; ++e) {
            if (used[static_cast<std::size_t>(e)]) continue;
            const double d = std::abs(v[e][axis]);
            if (d > best_dot) {
                best_dot = d;
                best = e;
            }
        }
        used[static_cast<std::size_t>(best)] = true;
        const double sign = v[best][axis] < 0.0 ? -1.0 : 1.0;
        pairing[axis] = sign * v[best];
        principal[static_cast<std::size_t>(axis)] = values[static_cast<std::size_t>(best)];
    }
    if (glm::determinant(pairing) < 0.0) pairing[2] = -pairing[2];
    const double turn_deg = std::acos(std::clamp((pairing[0][0] + pairing[1][1] + pairing[2][2] - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
    std::printf("[kit] principal moments (about the design axes' nearest principal axes): %.7f %.7f %.7f kg m^2\n", principal[0], principal[1], principal[2]);
    std::printf("[kit] the diagonal only (the products dropped): %.7f %.7f %.7f kg m^2\n", tensor[0][0], tensor[1][1], tensor[2][2]);
    std::printf("[kit] the principal axes turn %.3f degrees from the design axes\n", turn_deg);
    // The numbers tests/x500_model.hpp holds (kKitMass, kKitCom, kKitTensor), at 17 significant digits (a double round-trips: Core's compile pins them to 1e-9 relative).
    std::printf("[kit-const] mass %.17g\n[kit-const] com %.17g %.17g %.17g\n[kit-const] tensor xx %.17g yy %.17g zz %.17g xy %.17g xz %.17g yz %.17g\n", mass,
                centre.x, centre.y, centre.z, tensor[0][0], tensor[1][1], tensor[2][2], tensor[1][0], tensor[2][0], tensor[2][1]);
    EXPECT_NEAR(mass, 1.6326, 1e-4);
    EXPECT_GT(names.size(), 10u);
}

// L108: are the table's four in-place motor meshes (DJ-2216-KV880_1_1 .. _1_4) the library's ONE re-origined motor mesh turned about each motor's own vertical
// axis? For each, the vertices in order (same count assumed, checked): the turn about y that best maps the library's mesh onto it (least squares on the
// xz plane), then the translation, and the largest vertex distance left over: 0 for an exact rotation. The meshes are float32, so a residual near 1e-7 m at
// these coordinates is rounding.
TEST(X500KitInertia, TheTablesFourMotorsAreTurnsOfTheLibrarysOneMesh) {
    const char* enable = env_text("SPADE_X500_KIT_INERTIA");
    const char* store_text = env_text("SPADE_PART_STORE");
    if (enable == nullptr || store_text == nullptr) {
        GTEST_SKIP() << "set SPADE_X500_KIT_INERTIA=1 and SPADE_PART_STORE=<the shared part store>";
    }
    const std::filesystem::path store(store_text);
    const char* const kLibraryMotorSha = "dd3fdc640f9f30113f353ed08cb29f391a952e5d38f25ec17894c27c3dcc26f0";  // the library's motor part's store object (re-origined)
    const auto library = spade::load_mesh_file(store / kLibraryMotorSha / "DJ-2216-KV880_1_1.spmesh");
    ASSERT_TRUE(library.has_value()) << library.error().context;
    const std::span<const glm::vec3> v = library->mesh->positions();
    const YAML::Node doc = YAML::LoadFile(kTable);
    for (const YAML::Node& part : doc["parts"]) {
        if (part["kind"].as<std::string>() != "motor") continue;
        const YAML::Node ref = part["objects"][0]["store_ref"];
        const auto table = spade::load_mesh_file(store / ref["sha256"].as<std::string>() / ref["file"].as<std::string>());
        ASSERT_TRUE(table.has_value()) << table.error().context;
        const std::span<const glm::vec3> w = table->mesh->positions();
        std::printf("[kit-turn] %-18s %zu vertices (library %zu)", part["name"].as<std::string>().c_str(), w.size(), v.size());
        if (w.size() != v.size()) {
            std::printf(": not the same vertex count, so not the same mesh\n");
            continue;
        }
        glm::dvec3 vc(0.0), wc(0.0);
        for (std::size_t i = 0; i < v.size(); ++i) {
            vc += glm::dvec3(v[i]);
            wc += glm::dvec3(w[i]);
        }
        vc /= static_cast<double>(v.size());
        wc /= static_cast<double>(v.size());
        double sin_sum = 0.0, cos_sum = 0.0;
        for (std::size_t i = 0; i < v.size(); ++i) {
            const glm::dvec3 a = glm::dvec3(v[i]) - vc, b = glm::dvec3(w[i]) - wc;
            cos_sum += b.x * a.x + b.z * a.z;
            sin_sum += b.x * a.z - b.z * a.x;
        }
        const double theta = std::atan2(sin_sum, cos_sum);
        const double c = std::cos(theta), s = std::sin(theta);
        const auto turn = [&](const glm::dvec3& p) { return glm::dvec3(c * p.x + s * p.z, p.y, -s * p.x + c * p.z); };
        const glm::dvec3 t = wc - turn(vc);
        double worst = 0.0;
        for (std::size_t i = 0; i < v.size(); ++i) worst = std::max(worst, glm::length(turn(glm::dvec3(v[i])) + t - glm::dvec3(w[i])));
        std::printf(": turn about y %.9f degrees; translation (%.6f, %.6f, %.6f) m; largest vertex distance left %.3e m\n", theta * 180.0 / 3.14159265358979323846,
                    t.x, t.y, t.z, worst);
    }
}

}  // namespace

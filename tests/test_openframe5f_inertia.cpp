// ---------------------------------------------------------------------------
// test_openframe5f_inertia.cpp -- the OpenFrame-5F kit's mass, centre of mass and inertia from the build table's parts (Interface's
// tools/flow_movie/data/openframe5f_build/parts.yaml, 14 entries), each part at the table's pose, the way tests/test_x500_kit_inertia.cpp does the X500's:
// a part with store meshes (the frame's objects, the printed parts) from the meshes' own polyhedral mass properties (TriangleMesh::mass_properties(), exact
// for the shape) at uniform density scaled to the part's mass, each object at the table's pose composed with its own `offset`; a part the table gives as a
// box (the boards, the receiver, the pack) as a uniform box; a motor as a uniform cylinder (radius, length along the part's y); a rotor as a thin disc of its
// radius. It prints the composite's centre of mass, the full tensor about it, its principal moments and how far the principal axes turn from the design axes
// (the engine's model is principal-axes only, L88l), and the constants tests/openframe5f_model.hpp holds, at 17 significant digits.
//
// Equal density across a part's objects is assumed (the frame's carbon plates, aluminium standoffs and side plates share the maker's 170 g by volume), as
// the X500's frame was.
//
// OPT-IN: SPADE_OPENFRAME_INERTIA=1, and SPADE_PART_STORE the shared part store folder (<store>/<sha256>/<file>.spmesh; the CAD stays local, L88b).
// Without them the test skips: the meshes are not in git.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
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
const char* of_env(const char* name) {
    return std::getenv(name);
}
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

const std::string kTable = std::string(SPADE_REPO_DIR) + "/tools/flow_movie/data/openframe5f_build/parts.yaml";

struct Body {
    double mass = 0.0;
    glm::dvec3 centre{0.0};  // the table's frame
    glm::dmat3 tensor{0.0};  // about `centre`, the table's axes
};

glm::dvec3 vec3_of(const YAML::Node& n) {
    return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}

// The table's quaternions are w, x, y, z.
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

// A uniform solid cylinder of `radius` and `length` along y: m (3 r^2 + L^2) / 12 across, m r^2 / 2 about its axis.
glm::dmat3 cylinder_tensor(double mass, double radius, double length) {
    glm::dmat3 t(0.0);
    t[0][0] = mass * (3.0 * radius * radius + length * length) / 12.0;
    t[1][1] = 0.5 * mass * radius * radius;
    t[2][2] = t[0][0];
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

TEST(OpenFrame5fInertia, TheKitsMassCentreAndInertiaFromItsPartsMeshes) {
    if (of_env("SPADE_OPENFRAME_INERTIA") == nullptr || of_env("SPADE_PART_STORE") == nullptr) {
        GTEST_SKIP() << "set SPADE_OPENFRAME_INERTIA=1 and SPADE_PART_STORE=<the shared part store> to run the kit's inertia from its meshes";
    }
    const std::filesystem::path store(of_env("SPADE_PART_STORE"));
    const YAML::Node doc = YAML::LoadFile(kTable);
    std::vector<Body> bodies;
    for (const YAML::Node& part : doc["parts"]) {
        const std::string name = part["name"].as<std::string>();
        const std::string kind = part["kind"].as<std::string>();
        const double mass = part["mass_kg"].as<double>();
        const YAML::Node pose = part["pose"];
        const glm::dvec3 at = vec3_of(pose["position"]);
        const glm::dmat3 rot = rotation_of(pose["orientation"]);
        std::string basis;
        Body b;
        b.mass = mass;
        if (kind == "rotor") {
            // A thin disc of the prop's radius about its spin axis (y): 1/2 m R^2 about it, 1/4 m R^2 across.
            const double radius = part["radius"].as<double>();
            b.centre = at;
            b.tensor[0][0] = 0.25 * mass * radius * radius;
            b.tensor[1][1] = 0.5 * mass * radius * radius;
            b.tensor[2][2] = 0.25 * mass * radius * radius;
            basis = "thin disc R " + std::to_string(radius);
        } else if (part["objects"]) {
            // The part's meshes: each object's volume weights the part's mass (uniform density); an object's pose is the part's composed with its offset.
            struct Piece {
                double volume;
                glm::dvec3 centroid;
                glm::dmat3 inertia;  // per unit density, about the origin of the table's frame's axes at the centroid
            };
            std::vector<Piece> pieces;
            double volume = 0.0;
            for (const YAML::Node& object : part["objects"]) {
                const std::filesystem::path file =
                    store / object["store_ref"]["sha256"].as<std::string>() / object["store_ref"]["file"].as<std::string>();
                const auto loaded = spade::load_mesh_file(file);
                ASSERT_TRUE(loaded.has_value()) << file.string() << ": " << loaded.error().context;
                glm::dvec3 off_position(0.0);
                glm::dmat3 off_rot(1.0);
                if (object["offset"]) {
                    off_position = vec3_of(object["offset"]["position"]);
                    off_rot = rotation_of(object["offset"]["orientation"]);
                }
                const glm::dmat3 object_rot = rot * off_rot;
                const glm::dvec3 object_at = at + rot * off_position;
                const spade::MeshMassProperties& m = loaded->mesh->mass_properties();
                pieces.push_back({m.volume, object_at + object_rot * glm::dvec3(m.centroid), object_rot * glm::dmat3(m.inertia) * glm::transpose(object_rot)});
                volume += m.volume;
                // The object's box in the table's frame, for the flow's proxy solids (the kit model's plate stack and arms).
                glm::dvec3 lo(1.0e30), hi(-1.0e30);
                for (const glm::vec3& vertex : loaded->mesh->positions()) {
                    const glm::dvec3 w = object_at + object_rot * glm::dvec3(vertex);
                    lo = glm::min(lo, w);
                    hi = glm::max(hi, w);
                }
                std::printf("[kit-box] %-22s %-34s x %+.4f .. %+.4f  y %+.4f .. %+.4f  z %+.4f .. %+.4f m  volume %.4e m^3\n", name.c_str(),
                            object["store_ref"]["file"].as<std::string>().c_str(), lo.x, hi.x, lo.y, hi.y, lo.z, hi.z, m.volume);
            }
            for (const Piece& p : pieces) b.centre += (mass * p.volume / volume) * p.centroid;
            b.centre /= mass;
            for (const Piece& p : pieces) {
                const double mi = mass * p.volume / volume;
                b.tensor += (mi / p.volume) * p.inertia + parallel_axis(mi, p.centroid - b.centre);
            }
            basis = "meshes (" + std::to_string(pieces.size()) + ")";
        } else if (part["box"]) {
            b.centre = at;
            b.tensor = rot * box_tensor(mass, vec3_of(part["box"])) * glm::transpose(rot);
            basis = "box";
        } else if (part["cylinder"]) {
            b.centre = at;
            b.tensor = rot * cylinder_tensor(mass, part["cylinder"]["radius"].as<double>(), part["cylinder"]["length"].as<double>()) * glm::transpose(rot);
            basis = "cylinder";
        } else {
            ADD_FAILURE() << name << " (" << kind << ") has no meshes, box, cylinder or rotor";
            continue;
        }
        bodies.push_back(b);
        std::printf("[kit] %-22s %-10s %.4f kg  %s  centre (%.4f, %.4f, %.4f)\n", name.c_str(), kind.c_str(), mass, basis.c_str(), b.centre.x, b.centre.y,
                    b.centre.z);
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

    std::printf("[kit] total %.4f kg (the table's sum 0.5665); centre of mass (%.5f, %.5f, %.5f) m in the table's frame\n", mass, centre.x, centre.y, centre.z);
    std::printf("[kit] tensor about the centre of mass, kg m^2 (rows x, y, z):\n");
    for (int r = 0; r < 3; ++r) std::printf("[kit]   %+.6e %+.6e %+.6e\n", tensor[0][r], tensor[1][r], tensor[2][r]);
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
    std::printf("[kit] principal moments (about the design axes' nearest principal axes): %.7e %.7e %.7e kg m^2\n", principal[0], principal[1], principal[2]);
    std::printf("[kit] the principal axes turn %.3f degrees from the design axes\n", turn_deg);
    std::printf("[kit-const] mass %.17g\n[kit-const] com %.17g %.17g %.17g\n[kit-const] tensor xx %.17g yy %.17g zz %.17g xy %.17g xz %.17g yz %.17g\n", mass, centre.x,
                centre.y, centre.z, tensor[0][0], tensor[1][1], tensor[2][2], tensor[1][0], tensor[2][0], tensor[2][1]);
    EXPECT_NEAR(mass, 0.5665, 1e-4) << "the table's sum";
    EXPECT_EQ(bodies.size(), 14u) << "the table's 14 entries: the frame, the printed parts, FC, ESC, receiver, 4 motors, 4 rotors and the pack";
}

}  // namespace

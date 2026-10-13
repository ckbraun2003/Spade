#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstdint>
#include <ios>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "world/mesh.hpp"
#include "world/mesh_mass.hpp"

// ---------------------------------------------------------------------------
// world/mesh_mass.hpp: a mesh's volume, centroid and inertia per unit
// density, by Eberly's polyhedral integrals in double (the parts and meshes
// plan, Task B3; CORE-6).
//
// THE EXPECTATIONS are closed forms written here (TD-4): the textbook box and
// tetrahedra, the parallel-axis rule, and one independent evaluation by
// signed tetrahedra (a different formula from Eberly's). Each closed form
// takes the solid the mesh STORES: make_mesh() rounds every position to fp32
// once (Task B1), so a box of nominal side 0.3 is stored with a side that is
// not 0.3; where a fixture's coordinates are not fp32 values, the expectation
// reads the stored corners back (their differences are exact in double).
//
// THE TOLERANCE, kRel = 1e-15, relative to the quantity's own scale (the
// volume, or the tensor's trace for every tensor entry). Eberly's sums are a
// few dozen correctly rounded double operations per triangle. Against an
// exact rational evaluation of the same integrals over the same fp32
// positions (done outside the engine, for every fixture here), each output
// is within 1.1 ulp, and within 1e-16 of its scale. kRel, about 9 ulp,
// leaves room for the closed forms' own rounding in this file; any error in
// a formula, a sign or a shift is many orders larger.
// ---------------------------------------------------------------------------

namespace {

using spade::MeshInput;
using spade::MeshMassProperties;
using spade::MeshUnit;
using spade::TriangleMesh;

using MeshPtr = std::shared_ptr<const TriangleMesh>;

constexpr double kRel = 1e-15;

// The cube's twelve triangles, counter-clockwise seen from outside, its
// corners numbered x + 2y + 4z (as tests/test_mesh.cpp numbers them).
constexpr uint32_t kCubeTriangles[12][3] = {
    {0, 2, 1}, {1, 2, 3},  // z = lo, normal -z
    {4, 5, 6}, {5, 7, 6},  // z = hi, normal +z
    {0, 1, 4}, {1, 5, 4},  // y = lo, normal -y
    {2, 6, 3}, {3, 6, 7},  // y = hi, normal +y
    {0, 4, 2}, {2, 4, 6},  // x = lo, normal -x
    {1, 3, 5}, {3, 7, 5},  // x = hi, normal +x
};

// A box of full extents `size`, rotated by `rotation` about its centre and
// centred at `centre`: corner k is centre + rotation (corner_k * size), with
// corner_k's coordinates +-1/2, numbered x + 2y + 4z.
MeshInput box_mesh(glm::dvec3 size, glm::dvec3 centre, const glm::dmat3& rotation = glm::dmat3(1.0)) {
    MeshInput m;
    m.unit = MeshUnit::metre;
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::dvec3 corner{(v & 1u) != 0 ? 0.5 : -0.5, ((v >> 1) & 1u) != 0 ? 0.5 : -0.5,
                                ((v >> 2) & 1u) != 0 ? 0.5 : -0.5};
        m.positions.push_back(centre + rotation * (corner * size));
    }
    for (const auto& t : kCubeTriangles) m.indices.insert(m.indices.end(), {t[0], t[1], t[2]});
    return m;
}

// Every triangle reversed: as a shell inside another, a cavity.
MeshInput flipped(MeshInput m) {
    for (std::size_t t = 0; t < m.indices.size(); t += 3) std::swap(m.indices[t + 1], m.indices[t + 2]);
    return m;
}

// `a` then `b` as one input. The fixtures joined here share no position.
MeshInput joined(const MeshInput& a, const MeshInput& b) {
    MeshInput m = a;
    const uint32_t base = static_cast<uint32_t>(a.positions.size());
    m.positions.insert(m.positions.end(), b.positions.begin(), b.positions.end());
    for (const uint32_t i : b.indices) m.indices.push_back(base + i);
    return m;
}

// The regular tetrahedron s (+-1, +-1, +-1), an even count of minus signs,
// centred at `centre`: edge 2 sqrt(2) s.
MeshInput regular_tetrahedron(double s, glm::dvec3 centre) {
    MeshInput m;
    m.unit = MeshUnit::metre;
    for (const glm::dvec3 c : {glm::dvec3{1, 1, 1}, glm::dvec3{1, -1, -1}, glm::dvec3{-1, 1, -1},
                               glm::dvec3{-1, -1, 1}}) {
        m.positions.push_back(centre + s * c);
    }
    m.indices = {0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2};
    return m;
}

// The corner tetrahedron (o, o + s e_x, o + s e_y, o + s e_z).
MeshInput corner_tetrahedron(double s, glm::dvec3 o) {
    MeshInput m;
    m.unit = MeshUnit::metre;
    m.positions = {o, o + glm::dvec3{s, 0, 0}, o + glm::dvec3{0, s, 0}, o + glm::dvec3{0, 0, s}};
    m.indices = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
    return m;
}

MeshPtr build(const MeshInput& in, const char* name) {
    auto m = spade::make_mesh(in, name);
    EXPECT_TRUE(m.has_value()) << name << ": " << (m.has_value() ? std::string() : m.error().context);
    return m.has_value() ? *m : nullptr;
}

glm::dvec3 widen(const glm::vec3& v) {
    return {static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z)};
}

double trace(const glm::dmat3& m) {
    return m[0][0] + m[1][1] + m[2][2];
}

// The textbook box about its own centre: V (b^2 + c^2) / 12, and so on.
glm::dmat3 box_inertia(glm::dvec3 size) {
    const double v = size.x * size.y * size.z;
    glm::dmat3 i(0.0);
    i[0][0] = v * (size.y * size.y + size.z * size.z) / 12.0;
    i[1][1] = v * (size.x * size.x + size.z * size.z) / 12.0;
    i[2][2] = v * (size.x * size.x + size.y * size.y) / 12.0;
    return i;
}

// The parallel-axis rule: a solid's inertia about its centroid, moved to a
// point `d` away from it, V (|d|^2 E - d d^T) added.
glm::dmat3 shifted(const glm::dmat3& about_centroid, double volume, glm::dvec3 d) {
    return about_centroid + volume * (glm::dot(d, d) * glm::dmat3(1.0) - glm::outerProduct(d, d));
}

// Solids joined (sign +1) or carved out (sign -1), by the parallel-axis rule.
struct Solid {
    double volume;
    glm::dvec3 centroid;
    glm::dmat3 inertia;  // about its centroid
    double sign;
};

MeshMassProperties combine(const std::vector<Solid>& solids) {
    MeshMassProperties out;
    glm::dvec3 moment(0.0);
    for (const Solid& s : solids) {
        out.volume += s.sign * s.volume;
        moment += s.sign * s.volume * s.centroid;
    }
    out.centroid = moment / out.volume;
    for (const Solid& s : solids) {
        out.inertia += s.sign * shifted(s.inertia, s.volume, s.centroid - out.centroid);
    }
    return out;
}

// An independent evaluation (TD-4): every triangle (a, b, c) makes a signed
// tetrahedron with r, whose moments about r are the textbook ones, det =
// a . (b x c): volume det / 6, first moment det / 24 (a + b + c), second
// moment det / 120 (a a^T + b b^T + c c^T + s s^T), s = a + b + c. Summed,
// then moved to the centroid. Not Eberly's formula.
MeshMassProperties signed_tetrahedra(const TriangleMesh& mesh, glm::dvec3 r) {
    const auto p = mesh.positions();
    const auto idx = mesh.indices();
    double volume = 0.0;
    glm::dvec3 first(0.0);
    glm::dmat3 second(0.0);
    for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
        const glm::dvec3 a = widen(p[idx[3 * t]]) - r;
        const glm::dvec3 b = widen(p[idx[3 * t + 1]]) - r;
        const glm::dvec3 c = widen(p[idx[3 * t + 2]]) - r;
        const double det = glm::dot(a, glm::cross(b, c));
        const glm::dvec3 s = a + b + c;
        volume += det / 6.0;
        first += (det / 24.0) * s;
        second += (det / 120.0) * (glm::outerProduct(a, a) + glm::outerProduct(b, b) + glm::outerProduct(c, c) +
                                   glm::outerProduct(s, s));
    }
    const glm::dvec3 d = first / volume;
    const glm::dmat3 covariance = second - volume * glm::outerProduct(d, d);
    MeshMassProperties out;
    out.volume = volume;
    out.centroid = r + d;
    out.inertia = trace(covariance) * glm::dmat3(1.0) - covariance;
    return out;
}

void expect_tensor_near(const glm::dmat3& got, const glm::dmat3& want, double tolerance, const char* what) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(got[i][j], want[i][j], tolerance) << what << " [" << i << "][" << j << "]";
        }
    }
}

void expect_vec_near(glm::dvec3 got, glm::dvec3 want, double tolerance, const char* what) {
    for (int k = 0; k < 3; ++k) EXPECT_NEAR(got[k], want[k], tolerance) << what << " [" << k << "]";
}

bool same_bits(double a, double b) {
    return std::bit_cast<uint64_t>(a) == std::bit_cast<uint64_t>(b);
}

}  // namespace

TEST(MeshMass, AUnitCubeIsExact) {
    // [0, 1]^3. About its bounds' centre every corner is (+-1/2, +-1/2, +-1/2),
    // and every sum is exact in double, so each output is the double nearest
    // the true value: V = 1, the centroid (1/2, 1/2, 1/2), I = 1/6 on each
    // axis, and no product of inertia.
    const MeshPtr cube = build(box_mesh({1.0, 1.0, 1.0}, {0.5, 0.5, 0.5}), "cube");
    ASSERT_NE(cube, nullptr);
    const MeshMassProperties mp = spade::mesh_mass_properties(*cube);
    EXPECT_EQ(mp.volume, 1.0);
    EXPECT_EQ(mp.centroid, glm::dvec3(0.5));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_EQ(mp.inertia[i][j], i == j ? 1.0 / 6.0 : 0.0) << "[" << i << "][" << j << "]";
        }
    }
}

TEST(MeshMass, ABoxMatchesTheTextbookTensor) {
    // a x b x c = 0.3 x 0.1 x 0.2 m centred at (5, -2, 1): I_xx = V (b^2 + c^2) / 12,
    // and so on, with no product of inertia. The sides are the stored ones,
    // the differences of the fp32 corners (exact in double): fp32 holds 0.3
    // as 0.300000012 and 5.15 as 5.1500001, so the stored volume differs from
    // 0.006 by 8e-8 relative, which is Task B1's rounding, not this task's sums.
    const MeshPtr box = build(box_mesh({0.3, 0.1, 0.2}, {5.0, -2.0, 1.0}), "box");
    ASSERT_NE(box, nullptr);
    const auto p = box->positions();
    const glm::dvec3 lo = widen(p[0]);
    const glm::dvec3 hi = widen(p[7]);
    const glm::dvec3 size = hi - lo;
    const double v = size.x * size.y * size.z;
    const glm::dmat3 want = box_inertia(size);

    const MeshMassProperties mp = spade::mesh_mass_properties(*box);
    EXPECT_NEAR(mp.volume, v, kRel * v);
    expect_vec_near(mp.centroid, 0.5 * (lo + hi), kRel * glm::length(hi), "centroid");
    expect_tensor_near(mp.inertia, want, kRel * trace(want), "inertia");
    EXPECT_GT(mp.inertia[0][0], 0.0);
}

TEST(MeshMass, ARegularTetrahedron) {
    // Edge a = 2 sqrt(2) s, s = 1/4, centred at (1/2, 3/2, -3/4): every
    // coordinate is an fp32 value, so the stored solid is the textbook one.
    // V = a^3 / (6 sqrt 2); about the centroid I = V a^2 / 20 on every axis,
    // and no product of inertia (the tensor of a regular tetrahedron is
    // isotropic).
    const double s = 0.25;
    const glm::dvec3 centre{0.5, 1.5, -0.75};
    const MeshPtr tet = build(regular_tetrahedron(s, centre), "tetrahedron");
    ASSERT_NE(tet, nullptr);
    const double a = 2.0 * std::sqrt(2.0) * s;
    const double v = a * a * a / (6.0 * std::sqrt(2.0));
    const double moment = v * a * a / 20.0;

    const MeshMassProperties mp = spade::mesh_mass_properties(*tet);
    EXPECT_NEAR(mp.volume, v, kRel * v);
    expect_vec_near(mp.centroid, centre, kRel * glm::length(centre), "centroid");
    expect_tensor_near(mp.inertia, moment * glm::dmat3(1.0), kRel * 3.0 * moment, "inertia");
}

TEST(MeshMass, ACornerTetrahedronIsShiftedToItsCentroid) {
    // The tetrahedron (o, o + s e_x, o + s e_y, o + s e_z), s = 1/2,
    // o = (-1, 1/4, 2). Its centroid, o + s/4 (1, 1, 1), is not its bounds'
    // centre, o + s/2 (1, 1, 1), so the moments taken about the centre must
    // be moved to the centroid. By hand, about the corner o: V = s^3/6,
    // integral x^2 = s^5/60 and integral xy = s^5/120; the parallel-axis rule
    // takes V (s/4)^2 = s^5/96 off each, leaving s^5/160 and -s^5/480 about
    // the centroid. So I_xx = 2 s^5/160 = s^5/80 on each axis, and every
    // product of inertia is -(-s^5/480) = +s^5/480.
    const double s = 0.5;
    const glm::dvec3 o{-1.0, 0.25, 2.0};
    const MeshPtr tet = build(corner_tetrahedron(s, o), "corner");
    ASSERT_NE(tet, nullptr);
    const double s5 = s * s * s * s * s;
    glm::dmat3 want(0.0);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) want[i][j] = i == j ? s5 / 80.0 : s5 / 480.0;
    }

    const MeshMassProperties mp = spade::mesh_mass_properties(*tet);
    EXPECT_NEAR(mp.volume, s * s * s / 6.0, kRel * s * s * s / 6.0);
    expect_vec_near(mp.centroid, o + glm::dvec3(s / 4.0), kRel * glm::length(o), "centroid");
    expect_tensor_near(mp.inertia, want, kRel * trace(want), "inertia");
}

TEST(MeshMass, MovingTheMeshMovesTheCentroidOnly) {
    // 1. Translations fp32 holds exactly: the corner tetrahedron at the
    // origin and moved by t. The sums are taken about the bounds' centre,
    // which moves by exactly t, so every vertex is the same distance from it
    // and the volume and the inertia keep every bit. The centroid moves by t,
    // exactly here (every value is a short dyadic fraction).
    const MeshPtr home = build(corner_tetrahedron(0.5, {0.0, 0.0, 0.0}), "home");
    ASSERT_NE(home, nullptr);
    const MeshMassProperties at_home = spade::mesh_mass_properties(*home);
    EXPECT_EQ(at_home.centroid, glm::dvec3(0.125));
    for (const glm::dvec3 t : {glm::dvec3{1024.0, -512.0, 64.0}, glm::dvec3{3.5, -7.25, 0.125}}) {
        const MeshPtr moved = build(corner_tetrahedron(0.5, t), "moved");
        ASSERT_NE(moved, nullptr);
        const MeshMassProperties mp = spade::mesh_mass_properties(*moved);
        EXPECT_TRUE(same_bits(mp.volume, at_home.volume)) << mp.volume << " vs " << at_home.volume;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                EXPECT_TRUE(same_bits(mp.inertia[i][j], at_home.inertia[i][j])) << "[" << i << "][" << j << "]";
            }
        }
        EXPECT_EQ(mp.centroid, at_home.centroid + t);
    }

    // 2. A rotated and translated box: 0.3 x 0.1 x 0.2 m, turned by R, centred
    // at t = (5, -2, 1). fp32 rounds each rotated corner, so the stored solid
    // is the turned box to within delta = sqrt(3) 2^-22 m per corner (half an
    // fp32 ulp per coordinate, every |coordinate| < 8 m).
    const glm::dmat3 r = glm::mat3_cast(glm::normalize(glm::dquat(0.9, 0.3, -0.2, 0.25)));
    const glm::dvec3 size{0.3, 0.1, 0.2};
    const glm::dvec3 t{5.0, -2.0, 1.0};
    const MeshPtr turned = build(box_mesh(size, t, r), "turned");
    ASSERT_NE(turned, nullptr);
    const MeshMassProperties mp = spade::mesh_mass_properties(*turned);

    // (a) Exact for the solid stored: the signed tetrahedra from t agree to kRel.
    const MeshMassProperties tetra = signed_tetrahedra(*turned, t);
    EXPECT_NEAR(mp.volume, tetra.volume, kRel * tetra.volume);
    expect_vec_near(mp.centroid, tetra.centroid, kRel * glm::length(t), "centroid, against signed tetrahedra");
    expect_tensor_near(mp.inertia, tetra.inertia, kRel * trace(tetra.inertia), "inertia, against signed tetrahedra");

    // (b) The turned box: centroid t, tensor R I R^T, to fp32's rounding of the
    // corners. To first order, moving the surface by at most delta changes V
    // by at most area * delta, the first moment by area * rho * delta, and
    // each tensor entry by area * rho^2 * delta, rho the half-diagonal; the
    // bounds below are twice those. R I R^T's products of inertia are 4e-7
    // to 1.4e-5 m^5, so a wrong sign or a transposed R fails by far.
    const double delta = std::sqrt(3.0) * 0x1.0p-22;
    const double area = 2.0 * (size.x * size.y + size.y * size.z + size.z * size.x);
    const double rho2 = glm::dot(size, size) / 4.0;
    const double v = size.x * size.y * size.z;
    EXPECT_NEAR(mp.volume, v, 2.0 * area * delta);
    expect_vec_near(mp.centroid, t, 2.0 * area * std::sqrt(rho2) * delta / v, "centroid, against t");
    const glm::dmat3 rirt = r * box_inertia(size) * glm::transpose(r);
    expect_tensor_near(mp.inertia, rirt, 2.0 * area * rho2 * delta, "inertia, against R I R^T");
}

TEST(MeshMass, TwoShellsAdd) {
    // Two disjoint boxes in one mesh: the volumes add, the centroid is their
    // volume-weighted mean, and each tensor is moved to it by the
    // parallel-axis rule. The second box sits off every axis of the first,
    // so the sum has products of inertia. Every coordinate is an fp32 value.
    const glm::dvec3 size_a{1.0, 1.0, 1.0};
    const glm::dvec3 centre_a{0.5, 0.5, 0.5};
    const glm::dvec3 size_b{0.5, 0.25, 0.75};
    const glm::dvec3 centre_b{3.0, 1.25, -0.5};
    const MeshPtr two = build(joined(box_mesh(size_a, centre_a), box_mesh(size_b, centre_b)), "two");
    ASSERT_NE(two, nullptr);
    ASSERT_EQ(two->shell_count(), 2u);
    const MeshMassProperties want = combine({
        {size_a.x * size_a.y * size_a.z, centre_a, box_inertia(size_a), 1.0},
        {size_b.x * size_b.y * size_b.z, centre_b, box_inertia(size_b), 1.0},
    });
    ASSERT_NE(want.inertia[0][1], 0.0);

    const MeshMassProperties mp = spade::mesh_mass_properties(*two);
    EXPECT_NEAR(mp.volume, want.volume, kRel * want.volume);
    expect_vec_near(mp.centroid, want.centroid, kRel * glm::length(centre_b), "centroid");
    expect_tensor_near(mp.inertia, want.inertia, kRel * trace(want.inertia), "inertia");
}

TEST(MeshMass, ACavitySubtracts) {
    // The cube [0, 2]^3 with an inward cube [1/2, 1]^3 inside it: a cavity
    // off the centre. Its volume, its moment and its tensor come off the
    // outer cube's, with no special case: the reversed shell's integrals are
    // negative.
    const glm::dvec3 size_out{2.0, 2.0, 2.0};
    const glm::dvec3 centre_out{1.0, 1.0, 1.0};
    const glm::dvec3 size_in{0.5, 0.5, 0.5};
    const glm::dvec3 centre_in{0.75, 0.75, 0.75};
    const MeshPtr hollow =
        build(joined(box_mesh(size_out, centre_out), flipped(box_mesh(size_in, centre_in))), "hollow");
    ASSERT_NE(hollow, nullptr);
    const MeshMassProperties want = combine({
        {8.0, centre_out, box_inertia(size_out), 1.0},
        {0.125, centre_in, box_inertia(size_in), -1.0},
    });
    ASSERT_EQ(want.volume, 7.875);

    const MeshMassProperties mp = spade::mesh_mass_properties(*hollow);
    EXPECT_NEAR(mp.volume, want.volume, kRel * want.volume);
    expect_vec_near(mp.centroid, want.centroid, kRel * glm::length(centre_out), "centroid");
    expect_tensor_near(mp.inertia, want.inertia, kRel * trace(want.inertia), "inertia");
    EXPECT_GT(mp.centroid.x, 1.0) << "the cavity pushes the centroid away from it";
}

TEST(MeshMass, PropertiesArePerUnitDensityInMetres) {
    // A 10 mm cube declared in millimetres: the mesh holds metres, so its
    // side is s = float(0.01) m, V = s^3 m^3 and I = V (s^2 + s^2) / 12 =
    // s^5 / 6 m^5 on each axis -- per unit density, because neither the mesh
    // nor the result holds one (no part takes its mass from a mesh yet).
    MeshInput mm = box_mesh({10.0, 10.0, 10.0}, {5.0, 5.0, 5.0});
    mm.unit = MeshUnit::millimetre;
    const MeshPtr cube = build(mm, "mm");
    ASSERT_NE(cube, nullptr);
    const double s = static_cast<double>(cube->positions()[7].x);
    ASSERT_EQ(s, static_cast<double>(static_cast<float>(10.0 * 0.001)));

    // Three members and no more: this binding of three names compiles only
    // so, and a density field would break it.
    const auto& [volume, centroid, inertia] = spade::mesh_mass_properties(*cube);
    const double v = s * s * s;
    EXPECT_NEAR(volume, v, kRel * v);
    expect_vec_near(centroid, glm::dvec3(s / 2.0), kRel * s, "centroid");
    const double moment = v * s * s / 6.0;
    expect_tensor_near(inertia, moment * glm::dmat3(1.0), kRel * 3.0 * moment, "inertia");
}

TEST(MeshMass, OneInputGivesOneResultBitForBit) {
    // An irregular hexahedron, about 0.1 m across, every coordinate rounded
    // by fp32 and every face a bent quad: rounding at every step.
    MeshInput in;
    in.unit = MeshUnit::metre;
    in.positions = {
        {0.3713, -0.1221, 0.0457}, {0.4804, -0.1183, 0.0418}, {0.3673, -0.0219, 0.0469},
        {0.4693, -0.0154, 0.0389}, {0.3731, -0.1192, 0.1512}, {0.4717, -0.1229, 0.1423},
        {0.3722, -0.0187, 0.1537}, {0.4771, -0.0206, 0.1458},
    };
    for (const auto& t : kCubeTriangles) in.indices.insert(in.indices.end(), {t[0], t[1], t[2]});
    const MeshPtr one = build(in, "one");
    const MeshPtr two = build(in, "two");
    ASSERT_NE(one, nullptr);
    ASSERT_NE(two, nullptr);
    const MeshMassProperties a = spade::mesh_mass_properties(*one);
    const MeshMassProperties b = spade::mesh_mass_properties(*two);

    // The same input, twice: the same bits. The tensor is symmetric bit for bit.
    EXPECT_TRUE(same_bits(a.volume, b.volume));
    for (int k = 0; k < 3; ++k) EXPECT_TRUE(same_bits(a.centroid[k], b.centroid[k])) << k;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_TRUE(same_bits(a.inertia[i][j], b.inertia[i][j])) << "[" << i << "][" << j << "]";
            EXPECT_TRUE(same_bits(a.inertia[i][j], a.inertia[j][i])) << "[" << i << "][" << j << "]";
        }
    }

    // THE PIN. These bits were computed outside the engine by a second
    // implementation of the same pinned order (IEEE doubles with no fused
    // multiply-add, over the same fp32 positions, the triangles in index
    // order). This build and gcc's must both give them (TD-12): a change of
    // order, or a fused multiply-add, moves them. Against an exact rational
    // evaluation, the volume is within 8e-17 relative and each tensor entry
    // within 3e-17 of the trace, so these are correct as well as repeatable.
    EXPECT_TRUE(same_bits(a.volume, 0x1.1da45a8cd37c8p-10)) << std::hexfloat << a.volume;
    EXPECT_TRUE(same_bits(a.centroid.x, 0x1.b0120c21ae6d9p-2)) << std::hexfloat << a.centroid.x;
    EXPECT_TRUE(same_bits(a.centroid.y, -0x1.1a183ae0e8fcep-4)) << std::hexfloat << a.centroid.y;
    EXPECT_TRUE(same_bits(a.centroid.z, 0x1.87bd75ab06252p-4)) << std::hexfloat << a.centroid.z;
    EXPECT_TRUE(same_bits(a.inertia[0][0], 0x1.04de614a5b743p-19)) << std::hexfloat << a.inertia[0][0];
    EXPECT_TRUE(same_bits(a.inertia[1][1], 0x1.06ef4655fe399p-19)) << std::hexfloat << a.inertia[1][1];
    EXPECT_TRUE(same_bits(a.inertia[2][2], 0x1.fe6662e24494cp-20)) << std::hexfloat << a.inertia[2][2];
    EXPECT_TRUE(same_bits(a.inertia[0][1], 0x1.d0dd381b4db4cp-28)) << std::hexfloat << a.inertia[0][1];
    EXPECT_TRUE(same_bits(a.inertia[1][2], -0x1.c638495c2d7b6p-27)) << std::hexfloat << a.inertia[1][2];
    EXPECT_TRUE(same_bits(a.inertia[0][2], 0x1.bfe40bf5c3e61p-25)) << std::hexfloat << a.inertia[0][2];
}

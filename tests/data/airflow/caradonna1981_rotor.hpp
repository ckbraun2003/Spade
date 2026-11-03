#pragma once

// ---------------------------------------------------------------------------
// REFERENCE DATA: the hovering model rotor, blade-resolved (the demonstration
// plan's D3, its P9; the airflow spec's §11.2, C8's blade-resolved tier).
//
// SOURCE: F. X. Caradonna and C. Tung, "Experimental and Analytical Studies of
// a Model Helicopter Rotor in Hover", NASA TM-81232 (USAAVRADCOM TR-81-A-23),
// September 1981. NTRS 19820004169 (accession 82N12042), the scan read from
// https://ntrs.nasa.gov/api/citations/19820004169/downloads/19820004169.pdf:
// 60 pages, SHA-256
// 16c14789a5f65892b4ffb4e3628872723195c930f96680068ead7375e57208d2.
// A US Government work. Pages are the report's printed numbers, with the
// PDF's in brackets.
//   Table 17 (p. 26 [28]): the local pressure coefficient at collective
//     theta_c = 8 deg, 1250 rpm, tip Mach 0.439, at r/R 0.50, 0.68, 0.80,
//     0.89 and 0.96, each surface printed as -C_p against x/c, with each
//     station's C_L.
//   Section 2 (p. 2 [4]): two blades, NACA 0012, untwisted and untapered,
//     aspect ratio 6, half a degree of precone, cantilever-mounted; a large
//     chamber "with special ducting designed to eliminate room
//     recirculation".
//   Figure 1 (p. 42 [44]): the rotor's diameter 7.5 ft (2.286 m), so its
//     radius 1.143 m, half of it. (The figure's 45 in. is a vertical
//     dimension beside the hot-wire traverse, not the radius.)
//   Figure 4 (p. 45 [47]): C_T 0.00459 at this condition. Figure 6 (p. 47
//     [49]) prints 0.00460 for the same run: both are kept, below.
//   Figure 9 (p. 50 [52]): the tip vortex's descent z/R and radius r/R
//     against wake age at theta_c = 8 deg; the 1250 rpm triangles.
// The tip vortex path is plotted only. Every other value here is tabulated
// or printed in the text.
//
// METHOD (as Ghia's, docs/design/test-docs/2026-11-03-p9-rotor-data-plan.md;
// the review: docs/design/test-docs/2026-11-03-p9-caradonna-review.md):
// - Table 17 read by Test/Docs from the scan rendered at 300 dpi, every
//   faint cell again at 600 dpi (2026-11-03). The scan's OCR layer has no
//   usable text in the appendix tables, so it was no check. No independent
//   tabulated reproduction of Table 17 was found (SU2's test case holds the
//   mesh only), so the checks are the report's own: each station's
//   pressures integrate near its printed C_L, measured within 7.5% (0.925
//   to 1.055); the test's band is 10%
//   (AirflowCaradonnaData.ThePressuresIntegrateNearEachStationsLift), and the
//   lead's independent reading against the scan, every value and every
//   count.
// - Values are as printed, in the table's own notation, never corrected.
//   The faint cells are named in the review.
// - Figure 9 digitized from the scan at 600 dpi: the axes calibrated on their
//   own tick marks, piecewise between ticks (the 400-450 deg interval is
//   drawn 8% short), the triangles found by matching the legend's own
//   triangle and kept only where one stands clear of its neighbours. A
//   symbol is the uncertainty: +/- 4 deg of age and +/- 0.01 in z/R or r/R.
//   The overlapped triangles (near 90 and 372 deg in z/R, 235 deg in r/R)
//   are left out.
//
// CAVEATS, stated wherever the data is used:
// - Compressibility: at tip Mach 0.439 the Prandtl-Glauert factor
//   1/sqrt(1 - M^2) is 1.11 at the tip and 1.03 at r/R 0.5. An incompressible
//   solver's C_p may differ from the table by up to about 10% near the tip
//   from compressibility alone.
// - Reynolds number: the chord Reynolds number is about 2e6 at the tip and
//   1e6 at r/R 0.5 (omega R 149.6 m/s, the chord R/6, air's nu at 15 C; the
//   chamber's temperature is not printed). A drone propeller's is 1e4 to
//   1e5: the case validates the blade-resolved method, not transition at
//   drone scale.
// - The chord: 0.1905 m is derived from the radius and the aspect ratio
//   (R/c = 6, as the literature reads it), not printed.
// ---------------------------------------------------------------------------

#include <array>
#include <span>

namespace spade::testing::airflow_reference {

// The rotor (section 2, figure 1) and the condition (Table 17's title,
// figures 4 and 6).
inline constexpr int kCaradonnaBlades = 2;
inline constexpr double kCaradonnaRadiusM = 1.143;     // figure 1: half the 7.5 ft diameter
inline constexpr double kCaradonnaAspectRatio = 6.0;   // section 2
inline constexpr double kCaradonnaPreconeDeg = 0.5;    // section 2
inline constexpr double kCaradonnaCollectiveDeg = 8.0;
inline constexpr double kCaradonnaRpm = 1250.0;
inline constexpr double kCaradonnaTipMach = 0.439;
inline constexpr double kCaradonnaThrustCoefficientFig4 = 0.00459;
inline constexpr double kCaradonnaThrustCoefficientFig6 = 0.00460;

// One tap as Table 17 prints it: x/c, and -C_p on its surface.
struct CaradonnaTap {
    double x_c;
    double minus_cp;
};

// r/R = 0.50
inline constexpr CaradonnaTap kCaradonnaUpper050[] = {
    {0.00, -0.8538E+00}, {0.03, 0.8568E+00}, {0.12, 0.6735E+00}, {0.26, 0.5207E+00},
    {0.47, 0.3375E+00},  {0.69, 0.1949E+00}, {0.83, 0.9314E-01}};
inline constexpr CaradonnaTap kCaradonnaLower050[] = {
    {0.04, -0.9014E-01}, {0.20, 0.2357E+00}, {0.45, 0.1949E+00}, {0.69, 0.1033E+00}, {0.85, 0.4220E-01}};

// r/R = 0.68
inline constexpr CaradonnaTap kCaradonnaUpper068[] = {
    {0.02, 0.9420E+00}, {0.06, 0.8719E+00}, {0.10, 0.7667E+00}, {0.15, 0.6908E+00}, {0.19, 0.6324E+00},
    {0.23, 0.5856E+00}, {0.29, 0.5097E+00}, {0.33, 0.4805E+00}, {0.39, 0.3812E+00}, {0.44, 0.3753E+00},
    {0.52, 0.3052E+00}, {0.61, 0.2234E+00}, {0.73, 0.1592E+00}, {0.80, 0.1007E+00}};
inline constexpr CaradonnaTap kCaradonnaLower068[] = {
    {0.00, -0.8808E+00}, {0.07, -0.2523E-01}, {0.18, 0.2003E+00}, {0.28, 0.2168E+00},
    {0.38, 0.1939E+00},  {0.51, 0.1618E+00},  {0.57, 0.1305E+00}, {0.79, 0.1539E-01}};

// r/R = 0.80
inline constexpr CaradonnaTap kCaradonnaUpper080[] = {
    {0.00, 0.3750E+00}, {0.01, 0.9164E+00}, {0.04, 0.1091E+01}, {0.07, 0.8797E+00}, {0.09, 0.8553E+00},
    {0.13, 0.7617E+00}, {0.17, 0.7169E+00}, {0.21, 0.6477E+00}, {0.24, 0.5785E+00}, {0.30, 0.4849E+00},
    {0.35, 0.4483E+00}, {0.42, 0.3831E+00}, {0.50, 0.3180E+00}, {0.56, 0.2651E+00}, {0.65, 0.2000E+00},
    {0.76, 0.1145E+00}, {0.90, -0.1576E-01}};
inline constexpr CaradonnaTap kCaradonnaLower080[] = {
    {0.01, -0.5408E+00}, {0.02, -0.3780E+00}, {0.11, 0.9415E-01}, {0.14, 0.1593E+00}, {0.24, 0.1959E+00},
    {0.34, 0.1959E+00},  {0.57, 0.1306E+00},  {0.74, 0.6158E-01}, {0.90, -0.2390E-01}};

// r/R = 0.89
inline constexpr CaradonnaTap kCaradonnaUpper089[] = {
    {0.01, 0.1174E+01}, {0.03, 0.1184E+01}, {0.04, 0.1081E+01}, {0.06, 0.1005E+01}, {0.10, 0.8665E+00},
    {0.13, 0.7871E+00}, {0.17, 0.7180E+00}, {0.21, 0.6282E+00}, {0.26, 0.5418E+00}, {0.30, 0.5072E+00},
    {0.35, 0.4209E+00}, {0.42, 0.3690E+00}, {0.47, 0.3310E+00}, {0.52, 0.2723E+00}, {0.65, 0.1755E+00},
    {0.73, 0.1272E+00}, {0.80, 0.7190E-01}, {0.87, 0.1316E-01}};
inline constexpr CaradonnaTap kCaradonnaLower089[] = {
    {0.01, -0.7158E+00}, {0.04, -0.2771E+00}, {0.14, 0.1064E+00}, {0.28, 0.1721E+00}, {0.45, 0.1514E+00},
    {0.57, 0.1237E+00},  {0.69, 0.8917E-01},  {0.79, 0.2698E-01}, {0.90, -0.3521E-01}};

// r/R = 0.96
inline constexpr CaradonnaTap kCaradonnaUpper096[] = {
    {0.00, -0.5796E+00}, {0.02, 0.1116E+01}, {0.07, 0.9072E+00}, {0.12, 0.7171E+00}, {0.15, 0.6391E+00},
    {0.19, 0.5550E+00},  {0.23, 0.4615E+00}, {0.29, 0.4085E+00}, {0.33, 0.3649E+00}, {0.39, 0.3150E+00},
    {0.44, 0.2869E+00},  {0.50, 0.2589E+00}, {0.61, 0.1903E+00}, {0.69, 0.1467E+00}, {0.76, 0.1093E+00},
    {0.90, 0.9534E-02}};
inline constexpr CaradonnaTap kCaradonnaLower096[] = {
    {0.00, -0.1016E+01}, {0.07, -0.6215E-01}, {0.16, 0.1404E+00}, {0.24, 0.1623E+00}, {0.39, 0.1467E+00},
    {0.51, 0.1217E+00},  {0.63, 0.9992E-01},  {0.74, 0.5628E-01}, {0.85, 0.2823E-01}};

// Table 17's stations, inboard to the tip, each with its printed C_L.
struct CaradonnaStation {
    double r_over_R;
    std::span<const CaradonnaTap> upper;
    std::span<const CaradonnaTap> lower;
    double c_l;
};

inline constexpr std::array<CaradonnaStation, 5> kCaradonnaTable17 = {{
    {0.50, kCaradonnaUpper050, kCaradonnaLower050, .2345},
    {0.68, kCaradonnaUpper068, kCaradonnaLower068, .2815},
    {0.80, kCaradonnaUpper080, kCaradonnaLower080, .2886},
    {0.89, kCaradonnaUpper089, kCaradonnaLower089, .3143},
    {0.96, kCaradonnaUpper096, kCaradonnaLower096, .2683},
}};

// Figure 9's 1250 rpm triangles, digitized (METHOD above): wake age in
// degrees, and the descent below the rotor plane z/R (positive down) or the
// radius r/R. Each is +/- 4 deg and +/- 0.01. At 372 deg the two triangles
// sit about 0.025 inboard of the other speeds' points there; the clearer of
// the two, at 0.855, is kept as plotted.
struct CaradonnaVortexPoint {
    double age_deg;
    double value;
};

inline constexpr CaradonnaVortexPoint kCaradonnaVortexDescent[] = {{50.0, 0.007}, {162.0, 0.081}, {295.0, 0.212}};
inline constexpr CaradonnaVortexPoint kCaradonnaVortexRadius[] = {
    {49.0, 0.969}, {90.0, 0.946}, {162.0, 0.912}, {293.0, 0.880}, {372.0, 0.855}};
inline constexpr double kCaradonnaVortexAgeUncertaintyDeg = 4.0;
inline constexpr double kCaradonnaVortexPositionUncertainty = 0.01;

}  // namespace spade::testing::airflow_reference

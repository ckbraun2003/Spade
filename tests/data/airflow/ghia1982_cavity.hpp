#pragma once

// ---------------------------------------------------------------------------
// REFERENCE DATA: the lid-driven cavity (airflow spec §11.2, case C1).
//
// SOURCE: U. Ghia, K. N. Ghia and C. T. Shin, "High-Re solutions for
// incompressible flow using the Navier-Stokes equations and a multigrid
// method", J. Comput. Phys. 48 (1982) 387-411, doi:10.1016/0021-9991(82)90058-4.
//   Table I (p. 398):  u along the vertical line through the geometric centre
//                      (x = 0.5), at the tabulated y; the lid is y = 1, moving
//                      at u = 1.
//   Table II (p. 399): v along the horizontal line through the centre
//                      (y = 0.5), at the tabulated x.
// Re = 100 and 400 only: the columns V1 uses. Re 1000 is added, scan-verified,
// when V2 needs it.
//
// METHOD: read from a scan of the published pages, checked against the scan's
// OCR layer, and cross-checked with two independent reproductions (Ivan
// Pribec's ghiau/ghiav gists; Oceananigans.jl's lid-driven-cavity validation
// script), which agree with the scan in every value and coordinate. The
// Uppsala thesis's 0.05188 at Table II, point 65, Re 400 is a misreading of
// the printed 0.05186. Verification was done by the lead on 2026-10-05.
// Physics compared every value here with that reading by machine on
// 2026-10-12: 102 values, no difference. Test/Docs reviewed the same values
// against the same reading.
// ---------------------------------------------------------------------------
namespace spade::testing::airflow_reference {

inline constexpr double kGhiaY[17] = {1.0000, 0.9766, 0.9688, 0.9609, 0.9531, 0.8516, 0.7344, 0.6172, 0.5000,
                                      0.4531, 0.2813, 0.1719, 0.1016, 0.0703, 0.0625, 0.0547, 0.0000};
inline constexpr double kGhiaU100[17] = {1.00000,  0.84123,  0.78871,  0.73722,  0.68717,  0.23151,
                                         0.00332,  -0.13641, -0.20581, -0.21090, -0.15662, -0.10150,
                                         -0.06434, -0.04775, -0.04192, -0.03717, 0.00000};
inline constexpr double kGhiaU400[17] = {1.00000,  0.75837,  0.68439,  0.61756,  0.55892,  0.29093,
                                         0.16256,  0.02135,  -0.11477, -0.17119, -0.32726, -0.24299,
                                         -0.14612, -0.10338, -0.09266, -0.08186, 0.00000};

inline constexpr double kGhiaX[17] = {1.0000, 0.9688, 0.9609, 0.9531, 0.9453, 0.9063, 0.8594, 0.8047, 0.5000,
                                      0.2344, 0.2266, 0.1563, 0.0938, 0.0781, 0.0703, 0.0625, 0.0000};
inline constexpr double kGhiaV100[17] = {0.00000,  -0.05906, -0.07391, -0.08864, -0.10313, -0.16914,
                                         -0.22445, -0.24533, 0.05454,  0.17527,  0.17507,  0.16077,
                                         0.12317,  0.10890,  0.10091,  0.09233,  0.00000};
// kGhiaV400[5], Table II point 117 (x = 0.9063): -0.23827 is as printed. It
// breaks the trend between -0.22847 and -0.44993, and is suspected to be a
// typo in the original, but every source reproduces it, so it is kept as
// printed. Task 11 says whether its comparison leaves it out.
inline constexpr double kGhiaV400[17] = {0.00000,  -0.12146, -0.15663, -0.19254, -0.22847, -0.23827,
                                         -0.44993, -0.38598, 0.05186,  0.30174,  0.30203,  0.28124,
                                         0.22965,  0.20920,  0.19713,  0.18360,  0.00000};

// The index of Table II's point 117 in the arrays above (see kGhiaV400's note).
inline constexpr int kGhiaTableIIPoint117 = 5;

}  // namespace spade::testing::airflow_reference

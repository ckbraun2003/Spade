#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <vector>

#include "core/error.hpp"
#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// THE GRID'S STATE, SAVED AND RESTORED (the lead, 2026-10-13: the one
// serializer, TD-9; LiveRun's save and flow_movie's cache call it). The blob,
// all little-endian:
//
//   magic    8 bytes, "SPADEGRD"
//   version  u32, kGridBlobVersion
//   shape    u32 nx, ny, nz, and dx's float bits
//   lengths  u64 per array: u, v, w, p
//   data     u, v, w, then p, as fp32 bits
//
// u, v, w and p are everything a later fluid step reads that is not rebuilt.
// FluidScratch is rewritten each step, and its force accumulators are
// cleared by the step, so save only BETWEEN fluid steps. P2's solids -- the
// solid field, the fluid regions, the multigrid's weights -- are derived from
// the posed parts: a restore re-poses them and calls set_solids() before the
// next step. State a later part adds that is not derived grows the format
// through its version.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

inline constexpr uint32_t kGridBlobVersion = 1;
inline constexpr std::size_t kGridBlobHeaderBytes = 8 + 4 + 4 * 4 + 4 * 8;

// The blob of `g`. A host call: it allocates the blob.
[[nodiscard]] std::vector<std::byte> grid_to_bytes(const MacGrid& g);

// Restores `bytes` INTO `into`, a grid make_grid() built for the same case.
// Refused by name, with `into` untouched: a blob shorter than its header
// (invalid_argument); a foreign magic (invalid_argument); another version, or
// an nx, ny, nz, dx's bits or array length that is not into's, or a byte count
// its header does not give (schema_mismatch).
[[nodiscard]] Result<void> grid_from_bytes(std::span<const std::byte> bytes, MacGrid& into);

// The file: 8 bytes "SPADEGRF", a u32 version (1), the caller's step (u64),
// then the blob. save_grid() writes "<path>.tmp" and renames it over `path`,
// so a reader never sees half a file (io_error on a failed write or rename).
// load_grid() returns the step; refused naming the path: no file (not_found),
// a read that fails (io_error), a foreign or short file, and every refusal of
// grid_from_bytes().
[[nodiscard]] Result<void> save_grid(const std::filesystem::path& path, const MacGrid& g, uint64_t step);
[[nodiscard]] Result<uint64_t> load_grid(const std::filesystem::path& path, MacGrid& into);

}  // namespace spade::physics::airflow

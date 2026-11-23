#include "fc_link/run_support.hpp"

namespace spade::fc_link {

// RED: nothing is written yet; every helper refuses.

Result<uint64_t> measure_memory_peak_bytes() { return std::unexpected(Error{Code::internal, "measure_memory_peak_bytes: not written"}); }

Result<uint64_t> parse_peak_resident_kb(std::string_view) { return std::unexpected(Error{Code::internal, "parse_peak_resident_kb: not written"}); }

Result<std::string> hash_file(const std::filesystem::path&) { return std::unexpected(Error{Code::internal, "hash_file: not written"}); }

Result<fc_host::StageLatency> summarise_stage(std::string_view, std::span<const double>) {
    return std::unexpected(Error{Code::internal, "summarise_stage: not written"});
}

}  // namespace spade::fc_link

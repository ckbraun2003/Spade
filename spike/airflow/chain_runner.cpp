#include "spike/airflow/chain_runner.hpp"

// RED STUB (B1, red first): the API links and every call fails. Replaced by
// the implementation once the tests have been seen to fail.

namespace spade::airflow_gpu {

namespace {
[[nodiscard]] std::unexpected<Error> not_built() {
    return std::unexpected(Error{Code::internal, "airflow runner: not built yet (red stub)"});
}
}  // namespace

RunnerLimits read_runner_limits(const compute::VulkanContext&) { return RunnerLimits{}; }

uint64_t runner_budget_bytes(compute::VulkanContext&) { return 0; }

RunnerMemoryStats runner_memory_stats() noexcept { return RunnerMemoryStats{}; }

Result<std::unique_ptr<ChainRunner>> ChainRunner::create(compute::VulkanContext&, const RunnerDesc&) {
    return not_built();
}

ChainRunner::~ChainRunner() { destroy(); }

void ChainRunner::destroy() noexcept {}

Result<void> ChainRunner::upload(uint32_t, std::span<const std::byte>) { return not_built(); }

Result<void> ChainRunner::read(uint32_t, std::span<std::byte>) const { return not_built(); }

Result<uint32_t> ChainRunner::record(std::span<const compute::ChainCommand>) { return not_built(); }

Result<void> ChainRunner::submit(uint32_t, uint64_t) { return not_built(); }

Result<std::vector<uint64_t>> ChainRunner::durations_ns(uint32_t) const { return not_built(); }

compute::ChainCounts ChainRunner::recorded_counts(uint32_t) const noexcept { return compute::ChainCounts{}; }

}  // namespace spade::airflow_gpu

#include "fc_host/run_record.hpp"

namespace spade::fc_host {

// RED stubs: the run record says it is not built.
Result<RunRecord> build_run_record(const LockstepHost&, const RunIdentity&, const RunMeters&) {
    return std::unexpected(Error{Code::unavailable, "fc_host: build_run_record is not built yet"});
}

Result<void> write_run_folder(const RunRecord&, const std::filesystem::path&) {
    return std::unexpected(Error{Code::unavailable, "fc_host: write_run_folder is not built yet"});
}

}  // namespace spade::fc_host

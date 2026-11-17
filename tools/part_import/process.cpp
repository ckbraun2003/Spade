#include "process.hpp"

namespace spade::partimport {

ProcessResult run_process(const ProcessOptions&) {
    ProcessResult r;
    r.start_error = "not built";
    return r;
}

}  // namespace spade::partimport

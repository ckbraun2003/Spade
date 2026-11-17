#include "options.hpp"

namespace spade::partimport {

Expected<Options> parse_options(const std::vector<std::string>&) { return std::unexpected(Problem{"internal", "not built"}); }

const char* usage_text() { return ""; }

}  // namespace spade::partimport

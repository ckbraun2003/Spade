#include "report.hpp"

namespace spade::partimport {

const std::vector<CodeInfo>& all_codes() {
    static const std::vector<CodeInfo> none;
    return none;
}

const CodeInfo& code_info(std::string_view) {
    static const CodeInfo internal{"internal", kExitIo, ""};
    return internal;
}

std::string json_quote(std::string_view) { return {}; }
std::string json_number(double) { return {}; }

JsonObject& JsonObject::str(std::string_view, std::string_view) { return *this; }
JsonObject& JsonObject::num(std::string_view, double) { return *this; }
JsonObject& JsonObject::integer(std::string_view, std::int64_t) { return *this; }
JsonObject& JsonObject::boolean(std::string_view, bool) { return *this; }
JsonObject& JsonObject::null(std::string_view) { return *this; }
JsonObject& JsonObject::raw(std::string_view, std::string_view) { return *this; }
std::string JsonObject::line() const { return {}; }

}  // namespace spade::partimport

// ---------------------------------------------------------------------------
// scene/library_validate.cpp -- validate_text(): the existing loader for a kind run on a
// document's text (scene/library_validate.hpp).
// ---------------------------------------------------------------------------
#include "scene/library_validate.hpp"

#include <charconv>
#include <string>

#include "scene/library.hpp"
#include "scene/model_file.hpp"
#include "scene/object_file.hpp"
#include "scene/part_file.hpp"
#include "world/world_file.hpp"

namespace spade::scene {

namespace {

LibraryRefusal refuse(std::string code, std::string message, std::string field = {}, std::string path = {}) {
    LibraryRefusal r;
    r.code = std::move(code);
    r.message = std::move(message);
    r.field = std::move(field);
    r.path = std::move(path);
    return r;
}

const char* code_name(Code c) {
    switch (c) {
        case Code::invalid_argument: return "invalid_argument";
        case Code::capacity_exceeded: return "capacity_exceeded";
        case Code::not_found: return "not_found";
        case Code::io_error: return "io_error";
        case Code::schema_mismatch: return "schema_mismatch";
        case Code::internal: return "internal";
        case Code::unavailable: return "unavailable";
    }
    return "internal";
}

// The loaders end a message with " (line N, column M)" when they know the position.
void read_position(LibraryIssue& issue) {
    const std::string& m = issue.message;
    const std::size_t at = m.rfind(" (line ");
    if (at == std::string::npos) {
        return;
    }
    const char* p = m.data() + at + 7;
    const char* end = m.data() + m.size();
    uint32_t line = 0;
    const auto a = std::from_chars(p, end, line);
    if (a.ec != std::errc() || end - a.ptr < 9 || std::string_view(a.ptr, 9) != ", column ") {
        return;
    }
    uint32_t column = 0;
    const auto b = std::from_chars(a.ptr + 9, end, column);
    if (b.ec != std::errc() || b.ptr == end || *b.ptr != ')') {
        return;
    }
    issue.line = line;
    issue.column = column;
}

ValidationResult answer(const Error& e) {
    ValidationResult r;
    r.valid = false;
    LibraryIssue issue;
    issue.code = code_name(e.code);
    issue.message = e.context;
    read_position(issue);
    r.issues.push_back(std::move(issue));
    return r;
}

ValidationResult ok() {
    ValidationResult r;
    r.valid = true;
    return r;
}

template <class T>
ValidationResult of(const Result<T>& loaded) {
    return loaded ? ok() : answer(loaded.error());
}

}  // namespace

LibraryResult<ValidationResult> validate_text(std::string_view text, const ValidateOptions& options) {
    const std::string& as = options.as;
    if (!options.base_dir.empty() && !options.base_dir.is_absolute()) {
        return std::unexpected(refuse("relative_path", "base_dir must be an absolute path", "base_dir",
                                      options.base_dir.string()));
    }
    if (as == "model") {
        return of(model_from_yaml(text, options.base_dir));
    }
    if (as == "object") {
        return of(object_from_yaml(text));
    }
    if (as == "world") {
        return of(world_from_yaml(text));
    }
    if (as == "part.motor") {
        return of(part_from_yaml(text, PartKind::motor));
    }
    if (as == "part.prop") {
        return of(part_from_yaml(text, PartKind::prop));
    }
    if (as == "part.esc") {
        return of(part_from_yaml(text, PartKind::esc));
    }
    if (as == "part.battery") {
        return of(part_from_yaml(text, PartKind::battery));
    }
    if (as == "index") {
        return of(library_from_yaml(text));
    }
    if (as == "drone") {
        return std::unexpected(refuse("not_implemented", "validating a drone entry is not built yet", "as"));
    }
    return std::unexpected(refuse("invalid_request",
                                  "'as' is one of model, object, world, part.motor, part.prop, part.esc, part.battery, "
                                  "index (drone is not built yet); got '" + as + "'",
                                  "as"));
}

}  // namespace spade::scene

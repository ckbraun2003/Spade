#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

#include "library_json.hpp"
#include "spade_library.h"

// The C ABI (spade_library.h) over library_json.hpp. No exception crosses it, and every response is a malloc'd NUL-terminated string
// the caller frees with spade_lib_free.

struct spade_lib {
    spade::libc::LibraryPtr library;
};

namespace {

// Hands the response to the caller; false only when memory is out, which leaves *out null.
void give(const std::string& text, char** out) {
    if (out == nullptr) return;
    char* p = static_cast<char*>(std::malloc(text.size() + 1));
    if (p != nullptr) std::memcpy(p, text.c_str(), text.size() + 1);
    *out = p;
}

}  // namespace

extern "C" {

SPADE_LIBRARY_API const char* spade_lib_version(void) { return "spade.library/1.0"; }

SPADE_LIBRARY_API int spade_lib_open(const char* request_json, spade_lib** out, char** response_json) {
    if (out != nullptr) *out = nullptr;
    try {
        if (request_json == nullptr || out == nullptr) {
            give(spade::libc::misuse("open").json, response_json);
            return 2;
        }
        spade::libc::LibraryPtr library;
        spade::libc::Answer answer = spade::libc::open_library(request_json, library);
        if (answer.rc == 0) *out = new spade_lib{std::move(library)};
        give(answer.json, response_json);
        return answer.rc;
    } catch (const std::exception&) {
        if (response_json != nullptr) *response_json = nullptr;
        return 2;
    }
}

SPADE_LIBRARY_API int spade_lib_call(spade_lib* h, const char* op, const char* request_json, char** response_json) {
    try {
        if (h == nullptr || op == nullptr || request_json == nullptr || h->library == nullptr) {
            give(spade::libc::misuse(op == nullptr ? "" : op).json, response_json);
            return 2;
        }
        const spade::libc::Answer answer = spade::libc::call_library(*h->library, op, request_json);
        give(answer.json, response_json);
        return answer.rc;
    } catch (const std::exception&) {
        if (response_json != nullptr) *response_json = nullptr;
        return 2;
    }
}

SPADE_LIBRARY_API void spade_lib_close(spade_lib* h) { delete h; }

SPADE_LIBRARY_API void spade_lib_free(char* response_json) { std::free(response_json); }

}  // extern "C"

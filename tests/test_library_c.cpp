// ---------------------------------------------------------------------------
// test_library_c.cpp -- the library's C ABI and its JSON calls (tools/library_c; L88l, S4; the contract's sections 0 to 2 and 4).
//
//   LibraryC.EveryGoldenExchangeReplays       tests/data/library_contract/*.json: a request and its exact response per call, replayed against the
//                                             real library. KAT's test double replays the same files. `message` is ignored (it is for people
//                                             and may change); `$FIXTURES` is this folder, so the files carry no machine's path
//   LibraryC.TheGoldenFilesCoverTheSlice      every op the slice answers has a golden, and so does each refusal code it can raise
//   LibraryC.TheVersionStringIsTheFormat      spade_lib_version() is "spade.library/1.0"
//   LibraryC.MisuseAnswersTwo                 a null handle, op or request answers 2, with the refusal when there is somewhere to put it
//   LibraryC.AResponseIsFreedWithTheLibrarysFree   free(nullptr) and close(nullptr) are fine
//   LibraryC.ReadsRunConcurrentlyOnOneHandle  four threads find the same entry and get the same answer
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "../tools/library_c/spade_library.h"

namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;

fs::path fixtures() { return fs::path(SPADE_REPO_DIR) / "tests" / "data" / "library_contract"; }

std::string generic(const fs::path& p) {
    const std::u8string s = p.generic_u8string();
    return std::string(reinterpret_cast<const char*>(s.data()), s.size());
}

std::string slurp(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

std::string replace_all(std::string text, const std::string& from, const std::string& to) {
    for (std::size_t at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size())) text.replace(at, from.size(), to);
    return text;
}

// The golden's text with `$FIXTURES` made this machine's folder.
Json load_golden(const fs::path& file) {
    const std::string text = replace_all(slurp(file), "$FIXTURES", generic(fixtures()));
    Json j = Json::parse(text, nullptr, /*allow_exceptions=*/false);
    EXPECT_FALSE(j.is_discarded()) << file.string() << " is not JSON";
    return j;
}

// `message` is not compared: a refusal's text may change; its `code` and `field` may not.
void strip_messages(Json& j) {
    if (j.is_object()) {
        for (auto it = j.begin(); it != j.end(); ++it) {
            if (it.key() == "message") {
                it.value() = "";
            } else {
                strip_messages(it.value());
            }
        }
    } else if (j.is_array()) {
        for (Json& v : j) strip_messages(v);
    }
}

// A response, parsed; a path with a backslash (a native one the library echoed from the OS) is read as the forward-slash form.
Json take(char* response) {
    EXPECT_NE(response, nullptr);
    if (response == nullptr) return Json();
    Json j = Json::parse(std::string(response), nullptr, false);
    spade_lib_free(response);
    EXPECT_FALSE(j.is_discarded());
    return j;
}

struct Exchange {
    int rc = -1;
    Json response;
};

// Opens `open_request` (null: no handle at all), then makes the call. For op "open" the request IS the open request.
Exchange run(const Json& open_request, const std::string& op, const Json& request) {
    Exchange ex;
    spade_lib* handle = nullptr;
    char* response = nullptr;
    if (op == "open") {
        const std::string text = request.dump();
        ex.rc = spade_lib_open(text.c_str(), &handle, &response);
        ex.response = take(response);
        spade_lib_close(handle);
        return ex;
    }
    if (!open_request.is_null()) {
        const std::string text = open_request.dump();
        char* open_response = nullptr;
        const int open_rc = spade_lib_open(text.c_str(), &handle, &open_response);
        const Json opened = take(open_response);
        EXPECT_EQ(open_rc, 0) << opened.dump();
        if (open_rc != 0) return ex;
    }
    const std::string text = request.dump();
    ex.rc = spade_lib_call(handle, op.c_str(), text.c_str(), &response);
    ex.response = take(response);
    spade_lib_close(handle);
    return ex;
}

std::vector<fs::path> golden_files() {
    std::vector<fs::path> files;
    for (const fs::directory_entry& e : fs::directory_iterator(fixtures())) {
        if (e.is_regular_file() && e.path().extension() == ".json") files.push_back(e.path());
    }
    std::sort(files.begin(), files.end());
    return files;
}

TEST(LibraryC, EveryGoldenExchangeReplays) {
    const std::vector<fs::path> files = golden_files();
    ASSERT_FALSE(files.empty());
    for (const fs::path& file : files) {
        const Json golden = load_golden(file);
        ASSERT_TRUE(golden.is_object()) << file.string();
        const std::string name = file.filename().string();
        const Exchange ex = run(golden.value("open", Json()), golden.at("op").get<std::string>(), golden.at("request"));
        EXPECT_EQ(ex.rc, golden.at("rc").get<int>()) << name;
        Json want = golden.at("response");
        Json got = ex.response;
        strip_messages(want);
        strip_messages(got);
        // value equality: 0 and 0.0 are the same JSON number
        EXPECT_TRUE(got == want) << name << "\n got: " << got.dump(2) << "\nwant: " << want.dump(2);
    }
}

TEST(LibraryC, TheGoldenFilesCoverTheSlice) {
    std::set<std::string> ops;
    std::set<std::string> codes;
    for (const fs::path& file : golden_files()) {
        const Json golden = load_golden(file);
        ops.insert(golden.at("op").get<std::string>());
        const Json& response = golden.at("response");
        if (response.contains("refusal")) {
            codes.insert(response.at("refusal").at("code").get<std::string>());
            for (const Json& issue : response.at("refusal").at("issues")) codes.insert(issue.at("code").get<std::string>());
        }
        for (const Json& w : response.value("warnings", Json::array())) codes.insert(w.at("code").get<std::string>());
        // A per-item refusal in compose_placement counts too.
        for (const Json& item : response.value("result", Json::object()).value("items", Json::array())) {
            if (item.is_object() && item.contains("refusal") && item.at("refusal").is_object()) {
                codes.insert(item.at("refusal").at("code").get<std::string>());
            }
        }
    }
    for (const char* op : {"open", "list", "find", "resolve", "hash_text", "validate", "compose_placement"}) {
        EXPECT_TRUE(ops.count(op) != 0) << "no golden exchange for op " << op;
    }
    for (const char* code :
         {"invalid_request", "unknown_op", "unsupported_format", "not_implemented", "relative_path", "duplicate_root", "open_failed", "root_unavailable",
          "root_not_loaded", "bare_id", "entry_not_found", "store_file_missing", "hash_mismatch", "adjust_out_of_range"}) {
        EXPECT_TRUE(codes.count(code) != 0) << "no golden exchange raises " << code;
    }
}

TEST(LibraryC, TheVersionStringIsTheFormat) { EXPECT_STREQ(spade_lib_version(), "spade.library/1.0"); }

TEST(LibraryC, MisuseAnswersTwo) {
    char* response = nullptr;
    // a call before any open: no handle
    EXPECT_EQ(spade_lib_call(nullptr, "list", "{}", &response), 2);
    Json j = take(response);
    EXPECT_FALSE(j.at("ok").get<bool>());
    EXPECT_EQ(j.at("refusal").at("code"), "invalid_request");
    EXPECT_EQ(j.at("op"), "list");
    // a null response pointer: nothing is produced
    EXPECT_EQ(spade_lib_call(nullptr, "find", "{}", nullptr), 2);
    // open with a null request, a null out
    spade_lib* handle = reinterpret_cast<spade_lib*>(0x1);
    EXPECT_EQ(spade_lib_open(nullptr, &handle, &response), 2);
    EXPECT_EQ(handle, nullptr);
    EXPECT_EQ(take(response).at("refusal").at("code"), "invalid_request");
    EXPECT_EQ(spade_lib_open("{}", nullptr, &response), 2);
    spade_lib_free(response);
    // a real handle with a null op or request
    char* opened = nullptr;
    ASSERT_EQ(spade_lib_open(R"({"roots": []})", &handle, &opened), 0) << (opened != nullptr ? opened : "");
    spade_lib_free(opened);
    EXPECT_EQ(spade_lib_call(handle, nullptr, "{}", &response), 2);
    spade_lib_free(response);
    EXPECT_EQ(spade_lib_call(handle, "list", nullptr, &response), 2);
    spade_lib_free(response);
    spade_lib_close(handle);
}

TEST(LibraryC, AResponseIsFreedWithTheLibrarysFree) {
    spade_lib_free(nullptr);
    spade_lib_close(nullptr);
    SUCCEED();
}

TEST(LibraryC, ReadsRunConcurrentlyOnOneHandle) {
    const std::string request =
        replace_all(R"({"roots": [{"name": "default", "path": "$FIXTURES/roots/default"}]})", "$FIXTURES", generic(fixtures()));
    spade_lib* handle = nullptr;
    char* opened = nullptr;
    ASSERT_EQ(spade_lib_open(request.c_str(), &handle, &opened), 0) << (opened != nullptr ? opened : "");
    spade_lib_free(opened);
    std::vector<std::string> answers(4);
    std::vector<int> codes(4, -1);
    std::vector<std::thread> threads;
    for (std::size_t t = 0; t < answers.size(); ++t) {
        threads.emplace_back([&, t] {
            for (int i = 0; i < 20; ++i) {
                char* response = nullptr;
                codes[t] = spade_lib_call(handle, "find", R"({"id": "default:part.motor.fpv_2306"})", &response);
                if (response != nullptr) {
                    answers[t] = response;
                    spade_lib_free(response);
                }
            }
        });
    }
    for (std::thread& t : threads) t.join();
    spade_lib_close(handle);
    for (std::size_t t = 0; t < answers.size(); ++t) {
        EXPECT_EQ(codes[t], 0);
        EXPECT_EQ(answers[t], answers[0]);
    }
    EXPECT_FALSE(answers[0].empty());
}

}  // namespace

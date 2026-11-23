#pragma once

// ---------------------------------------------------------------------------
// fc_ulog_check.hpp -- K4.log: is a PX4 ULog file WELL-FORMED? (O140/O144, docs/design/test-docs/plans/2026-11-21-fc-tests.md section 3.)
//
// A reader written from the ULog file-format specification and nothing of PX4's code:
//   PX4 Guide, "ULog File Format" (https://docs.px4.io/v1.17/en/dev_log/ulog_file_format; read 2026-11-22, HTML).
// What the reader checks, and ALL it claims (it does not say PX4's data is right):
//   * the 16-byte header: the 7 magic bytes, the version byte (1), the start time;
//   * every message by its framing {u16 msg_size, u8 msg_type, body}, to the last byte of the file: none is cut off, and the file ends on a message boundary;
//   * the message types are the spec's: B F I M P Q (definitions) and A R D L C S O (data). A type NOT on that list, with its framing intact, is
//     SKIPPED, counted and named, and the answer is `unread` (never a pass, never a failure): the spec's requirements say a parser "must ignore unknown
//     messages", and a new message type can be announced by a new compat flag bit that existing parsers ignore. A broken framing anywhere still fails;
//   * the flag bits message `B` is the first message; its incompatible-flag bits are ones this reader knows (the spec says a reader must refuse others);
//     appended data (incompat bit 0) is NOT read: the answer is `unread`, never a pass;
//   * the definitions section holds only B F I M P Q and ends at the first A or L; the data section holds no B or F;
//   * bodies are long enough for their fixed fields (I, M, P, Q key lengths inside the body; A, D, L, C, S, O, R sizes), a format `F` is `name:field;...`,
//     a subscription `A` names a format the file defined and a message id not subscribed before, a data message `D` carries a subscribed id, the sync
//     message's 8 magic bytes are the spec's;
//   * the log holds at least one data message.
// What it does NOT check: the field layouts of the logged data (a D body's size against its format), timestamps, parameter values, the firmware
// version, that a message is the one PX4 should have logged. Those are K4's later steps.
// Since K4.ekf: it also KEEPS the values of three topics (instance 0: estimator_status's two flag fields, every float of estimator_innovation_test_ratios,
// every bool of failsafe_flags) and the level, stamp and text length of every logged string. Field names, types and offsets come from the log's own
// format definitions (the `F` messages), never from PX4's source; a field of a type that is not a basic one makes the topic `problem`, not guessed.
// Sources, said as they are: the page was read through a fetch tool that returns the specification only as a model-extracted SUMMARY (the first 39,938
// characters of the page are navigation), so no sentence below is a verbatim quote, and the first read of it lacked the bodies of L, C, S and O, the sync
// magic and the unknown-type rule. A second, targeted read of the same page (2026-11-22) gave, in that summary: C = log_level u8, tag u16, timestamp u64,
// text (msg_size - 11); O = a u16 duration in ms; S = the 8 sync magic bytes 2F 73 13 20 25 0C BB 12; and the requirement "must ignore unknown messages".
// The sizes used here (L 9 + text: level u8 + timestamp u64; C 11 + text; O 2; S 8) agree with it. Real data: two real PX4 v1.17.0 logs (the live runs
// fc-live-1 and fc-live-2, about 12 MB each, 109,160 and 108,286 messages) read well-formed to their last byte (SPADE_FC_ULOG in test_fc_run_folder.cpp);
// they hold B I F P Q M A D L and S only. So the sync magic (79 and 78 sync messages, every one matching) and the logged string's minimum size (12 each,
// sizes 51 to 96) are CONFIRMED by real data. The C and O layouts rest on the page summary alone: no real log here holds one (no C, no O, no R).
//
// A detail never quotes the file's bytes (a key, a value, a string): it names a byte offset, a message type and a size.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace spade::fc {

// The estimator-health topics the reader keeps the VALUES of (instance 0 only), for K4.ekf.* in fc_run_folder_check.hpp. Their field names and types are read
// from the log's own format definitions (the `F` messages), never from this file: the names below are the topics'.
inline constexpr std::string_view kEkfStatusTopic = "estimator_status";
inline constexpr std::string_view kEkfRatiosTopic = "estimator_innovation_test_ratios";
inline constexpr std::string_view kFailsafeTopic = "failsafe_flags";

struct UlogResult {
    enum class Kind { well_formed, malformed, unread };
    Kind kind = Kind::malformed;
    std::string detail;
    uint64_t messages = 0;       // every message after the header
    uint64_t definitions = 0;    // those before the data section began
    uint64_t data_messages = 0;  // D messages
    uint64_t subscriptions = 0;  // A messages
    uint64_t syncs = 0;          // S messages
    uint64_t dropouts = 0;       // O messages
    uint64_t unknown_messages = 0;  // messages of a type the spec's list lacks, skipped (framing intact)
    std::string unknown_types;      // those types, once each, as 'Z' or 0x01
    // The simulated-time span, read from the FIRST field of every data message whose format declares it `uint64_t timestamp` (the spec's rule for
    // logged data). A stamp of 0 (a message published with the field never set) is counted per format and kept OUT of the span; a format whose first
    // field is not that is counted as "not timed" and not read. The header's start time is NOT used (it is the logger's start, not the data's).
    bool has_time = false;           // at least one nonzero stamp was read
    uint64_t first_stamp_us = 0;     // the smallest nonzero stamp
    uint64_t last_stamp_us = 0;      // the largest nonzero stamp
    uint64_t timed_messages = 0;     // D messages of a format whose first field is `uint64_t timestamp`
    uint64_t zero_stamps = 0;        // of those, stamped 0
    std::string zero_formats;        // "name n, name n" for the formats that carried a 0 stamp
    uint64_t untimed_messages = 0;   // D messages of a format whose first field is not `uint64_t timestamp`
    std::string untimed_formats;     // those formats, once each
    uint64_t header_start_us = 0;    // the header's u64 (reported, never used as the span's start)
    // The firmware's identity, from the information messages `I`: `char[N] ver_sw` (the git commit) and `uint32_t ver_sw_release` (PX4's own encoding
    // 0xMMmmPPtt: major, minor, patch, release type 0xFF for a release; read from the real logs, not from the ULog page).
    bool has_ver_sw = false;
    std::string ver_sw;
    bool has_ver_sw_release = false;
    uint32_t ver_sw_release = 0;
    // The parameters, from the parameter messages `P`: `float NAME` and `int32_t NAME` with a 4-byte value (other types are not kept).
    struct Param {
        bool is_float = false;
        int32_t i = 0;
        float f = 0.0F;
    };
    std::map<std::string, Param> params;
    // The sample cadence: per subscription (one topic instance), the nonzero stamps of its timed data messages, in file order. `gcd_us` is the
    // greatest common divisor of the gaps between consecutive stamps, so every gap is a multiple of a step exactly when gcd_us is.
    struct Stream {
        std::string name;        // the format's name
        uint8_t multi_id = 0;    // the subscription's instance
        uint64_t n = 0;          // samples (nonzero stamps)
        uint64_t first_us = 0;
        uint64_t last_us = 0;
        uint64_t gcd_us = 0;     // 0 until two samples
        uint64_t min_gap_us = ~uint64_t{0};
        uint64_t max_gap_us = 0;
        bool monotone = true;    // no stamp earlier than the one before it
        uint64_t repeats = 0;    // gaps of 0 (a stamp repeated)
        // For each distinct gap between consecutive stamps, the stamp it FOLLOWED the first time it occurred: so the first stamp after which a gap of
        // a given size appears is known (the earliest missing step of a stream that should have none).
        std::map<uint64_t, uint64_t> first_at_gap;
    };
    std::vector<Stream> streams;

    // The estimator's health, from the VALUES of instance 0 of three topics and from the logged strings. A series is `present` when the log subscribes to the
    // topic's instance 0; `problem` says why it cannot be read (a field missing or of another type, a type this reader does not read, a data message too
    // short for the fields it needs); a series with a problem holds no samples. A sample is kept only when its timestamp is nonzero.
    struct EkfStatusSample {
        uint64_t t_us = 0;
        uint32_t fault_flags = 0;     // estimator_status.filter_fault_flags
        uint16_t solution_flags = 0;  // estimator_status.solution_status_flags
    };
    struct EkfStatusSeries {
        bool present = false;
        std::string problem;
        std::vector<EkfStatusSample> samples;
    };
    struct RatioSeries {  // estimator_innovation_test_ratios: every float field, an array by index ("gravity[2]")
        bool present = false;
        std::string problem;
        std::vector<std::string> names;
        std::vector<uint64_t> t_us;
        std::vector<std::vector<float>> rows;
    };
    struct FlagSeries {  // failsafe_flags: every bool field (the uint32 mode masks and the uint8 battery_warning are not flags)
        bool present = false;
        std::string problem;
        std::vector<std::string> names;
        std::vector<uint64_t> t_us;
        std::vector<std::vector<uint8_t>> rows;
    };
    struct LogLine {  // an `L` or a `C` message: the level byte, the stamp, the length of the text (the text itself is never kept)
        uint8_t level = 0;
        uint64_t t_us = 0;
        uint64_t text_bytes = 0;
    };
    EkfStatusSeries ekf_status;
    RatioSeries test_ratios;
    FlagSeries failsafe;
    std::vector<LogLine> log_lines;
};

namespace ulog_detail {

[[nodiscard]] inline uint16_t rd16(std::string_view b, size_t at) {
    return static_cast<uint16_t>(static_cast<uint8_t>(b[at]) | (static_cast<uint16_t>(static_cast<uint8_t>(b[at + 1])) << 8));
}

[[nodiscard]] inline uint32_t rd32(std::string_view b, size_t at) {
    uint32_t v = 0;
    for (size_t i = 0; i < 4; ++i) v |= static_cast<uint32_t>(static_cast<uint8_t>(b[at + i])) << (8 * i);
    return v;
}

[[nodiscard]] inline uint64_t rd64(std::string_view b, size_t at) {
    uint64_t v = 0;
    for (size_t i = 0; i < 8; ++i) v |= static_cast<uint64_t>(static_cast<uint8_t>(b[at + i])) << (8 * i);
    return v;
}

[[nodiscard]] inline bool valid_format_name_char(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '/';
}

// "message_name:field0;field1;" : a name of the allowed characters, then at least one field, the text ending with ';'. Returns the name, or empty.
[[nodiscard]] inline std::string format_name_of(std::string_view text) {
    const size_t colon = text.find(':');
    if (colon == std::string_view::npos || colon == 0 || colon + 1 >= text.size() || text.back() != ';') return {};
    for (size_t i = 0; i < colon; ++i)
        if (!valid_format_name_char(text[i])) return {};
    return std::string(text.substr(0, colon));
}

[[nodiscard]] inline std::string type_text(char type) {
    const unsigned char u = static_cast<unsigned char>(type);
    if ((u >= 'A' && u <= 'Z') || (u >= 'a' && u <= 'z') || (u >= '0' && u <= '9')) return std::string("'") + type + "'";
    static const char* hex = "0123456789abcdef";
    return std::string("0x") + hex[u >> 4] + hex[u & 15];
}

// A format's fields with their byte offsets in a data message's body (after the 2-byte message id). A trailing run of padding is NOT stored in a data message,
// so a body can end before the format does; a field is read only if its bytes are there.
struct FieldLayout {
    std::string type;  // "uint32_t", "float", "bool" ...
    uint32_t count = 1;
    std::string name;
    size_t offset = 0;
    size_t size = 0;  // of one element
};

[[nodiscard]] inline size_t basic_type_size(std::string_view t) {
    if (t == "int8_t" || t == "uint8_t" || t == "bool" || t == "char") return 1;
    if (t == "int16_t" || t == "uint16_t") return 2;
    if (t == "int32_t" || t == "uint32_t" || t == "float") return 4;
    if (t == "int64_t" || t == "uint64_t" || t == "double") return 8;
    return 0;
}

// "name:type field;type[n] field;..." -> the fields. False, with the reason, when a field is of a type that is not a basic one (a nested message) or is
// malformed: the offsets after it cannot be known.
[[nodiscard]] inline bool parse_layout(std::string_view format, std::vector<FieldLayout>* out, std::string* why) {
    const size_t colon = format.find(':');
    if (colon == std::string_view::npos) {
        *why = "the format has no fields";
        return false;
    }
    size_t at = colon + 1;
    size_t offset = 0;
    while (at < format.size()) {
        const size_t semi = format.find(';', at);
        if (semi == std::string_view::npos) break;
        const std::string_view decl = format.substr(at, semi - at);
        at = semi + 1;
        if (decl.empty()) continue;
        const size_t space = decl.find(' ');
        if (space == std::string_view::npos || space + 1 >= decl.size()) {
            *why = "a field that is not `type name`";
            return false;
        }
        FieldLayout f;
        std::string_view type = decl.substr(0, space);
        f.name = std::string(decl.substr(space + 1));
        const size_t bracket = type.find('[');
        if (bracket != std::string_view::npos) {
            if (type.back() != ']' || bracket + 2 > type.size()) {
                *why = "an array field with a malformed length";
                return false;
            }
            uint32_t n = 0;
            for (const char c : type.substr(bracket + 1, type.size() - bracket - 2)) {
                if (c < '0' || c > '9') {
                    *why = "an array field with a malformed length";
                    return false;
                }
                n = n * 10 + static_cast<uint32_t>(c - '0');
            }
            f.count = n;
            type = type.substr(0, bracket);
        }
        f.type = std::string(type);
        f.size = basic_type_size(f.type);
        if (f.size == 0 || f.count == 0) {
            *why = "a field of a type this reader does not read (" + f.type + ")";
            return false;
        }
        f.offset = offset;
        offset += f.size * f.count;
        out->push_back(std::move(f));
    }
    return true;
}

[[nodiscard]] inline uint64_t rd_uint(std::string_view b, size_t at, size_t size) {
    uint64_t v = 0;
    for (size_t i = 0; i < size; ++i) v |= static_cast<uint64_t>(static_cast<uint8_t>(b[at + i])) << (8 * i);
    return v;
}

[[nodiscard]] inline const FieldLayout* field_named(const std::vector<FieldLayout>& fields, std::string_view name) {
    for (const FieldLayout& f : fields)
        if (f.name == name) return &f;
    return nullptr;
}

}  // namespace ulog_detail

[[nodiscard]] inline UlogResult read_ulog(std::string_view b) {
    using namespace ulog_detail;
    UlogResult r;
    const auto bad = [&r](const std::string& why, size_t at) {
        r.kind = UlogResult::Kind::malformed;
        r.detail = why + " (message at byte " + std::to_string(at) + ")";
        return r;
    };
    static constexpr char kMagic[7] = {'U', 'L', 'o', 'g', '\x01', '\x12', '\x35'};
    static constexpr unsigned char kSync[8] = {0x2F, 0x73, 0x13, 0x20, 0x25, 0x0C, 0xBB, 0x12};  // UNVERIFIED (see the header)
    if (b.size() < 16) return bad("the file is shorter than the 16-byte header (" + std::to_string(b.size()) + " bytes)", 0);
    if (b.substr(0, 7) != std::string_view(kMagic, 7)) return bad("the file does not open with the ULog magic", 0);
    if (static_cast<uint8_t>(b[7]) != 1) return bad("the version byte is " + std::to_string(static_cast<uint8_t>(b[7])) + ", not the 1 this reader reads", 7);

    size_t pos = 16;
    bool data_section = false;
    bool first = true;
    std::set<std::string> formats;
    std::map<uint16_t, bool> subscribed;
    std::map<std::string, bool> timed;                 // format name -> its first field is `uint64_t timestamp`
    std::map<uint16_t, std::string> subscribed_name;   // message id -> format name
    std::map<std::string, uint64_t> zero_by_format;
    std::map<uint16_t, uint8_t> subscribed_multi;
    std::map<uint16_t, UlogResult::Stream> stream_of;
    std::map<std::string, std::string> format_text;  // format name -> its whole `name:field;...` text
    // The estimator-health topics' instance-0 subscriptions: how to read their data messages.
    struct Wanted {
        enum class Kind { status, ratios, flags } kind = Kind::status;
        std::vector<FieldLayout> fields;
        size_t fault_at = 0;
        size_t solution_at = 0;
        std::vector<size_t> picked;  // ratios, flags: the float / bool fields, by index into `fields`
    };
    std::map<uint16_t, Wanted> wanted;
    r.header_start_us = rd64(b, 8);
    while (pos < b.size()) {
        if (b.size() - pos < 3) return bad("a message header is cut off by the end of the file", pos);
        const size_t size = rd16(b, pos);
        const char type = b[pos + 2];
        const size_t body_at = pos + 3;
        if (b.size() - body_at < size)
            return bad("a " + type_text(type) + " message of " + std::to_string(size) + " bytes is cut off: " + std::to_string(b.size() - body_at) + " remain", pos);
        const std::string_view m = b.substr(body_at, size);
        const bool known = std::string_view("BFIMPQARDLCSO").find(type) != std::string_view::npos;
        if (first && type != 'B') return bad("the first message is " + type_text(type) + ", not the flag bits message 'B'", pos);
        if (!known) {
            // The spec's requirements section says a parser "must ignore unknown messages", and a new message type can be announced by a new compat
            // flag bit that existing parsers ignore. So an unknown type with intact framing is neither a pass nor a failure: it is skipped, counted and
            // named, and the answer is `unread` (whether PX4 may log it is not this reader's to say).
            ++r.unknown_messages;
            const std::string t = type_text(type);
            if (r.unknown_types.find(t) == std::string::npos) r.unknown_types += (r.unknown_types.empty() ? "" : ", ") + t;
            pos = body_at + size;
            continue;
        }
        if (type == 'A' || type == 'L') data_section = true;
        const bool def_type = std::string_view("BFIMPQ").find(type) != std::string_view::npos;
        if (!def_type && !data_section && type != 'A' && type != 'L')
            return bad("a " + type_text(type) + " message in the definitions section (the data section opens at the first A or L)", pos);
        if (data_section && (type == 'B' || type == 'F')) return bad("a " + type_text(type) + " message in the data section", pos);
        ++r.messages;
        if (!data_section) ++r.definitions;

        switch (type) {
            case 'B': {
                if (!first) return bad("a flag bits message that is not the first", pos);
                if (size < 40) return bad("the flag bits message is " + std::to_string(size) + " bytes, under 40", pos);
                bool other_incompat = (static_cast<uint8_t>(m[8]) & 0xFE) != 0;
                for (size_t i = 9; i < 16; ++i) other_incompat = other_incompat || m[i] != 0;
                if (other_incompat) return bad("incompatible flag bits this reader does not know are set (the spec says to refuse)", pos);
                bool appended = (static_cast<uint8_t>(m[8]) & 1) != 0;
                for (size_t i = 16; i < 40; ++i) appended = appended || m[i] != 0;
                if (appended) {
                    r.kind = UlogResult::Kind::unread;
                    r.detail = "the log has appended data (incompatible flag bit 0): this reader does not read it";
                    return r;
                }
                break;
            }
            case 'F':
                if (format_name_of(m).empty()) return bad("a format message that is not `name:field;...` with a valid name", pos);
                formats.insert(format_name_of(m));
                format_text[format_name_of(m)] = std::string(m);
                {
                    const size_t colon = m.find(':');
                    const size_t semi = m.find(';', colon);
                    timed[format_name_of(m)] = m.substr(colon + 1, semi - colon - 1) == "uint64_t timestamp";
                }
                break;
            case 'I':
            case 'P':
                if (size < 1 || m[0] == 0 || static_cast<size_t>(static_cast<uint8_t>(m[0])) + 1 > size)
                    return bad(std::string("an ") + type + " message whose key length is zero or runs past the message", pos);
                {
                    const std::string_view key = m.substr(1, static_cast<uint8_t>(m[0]));
                    const std::string_view val = m.substr(1 + static_cast<uint8_t>(m[0]));
                    if (type == 'I') {
                        if (key.substr(0, 5) == "char[" && key.size() > 7 && key.substr(key.size() - 7) == " ver_sw") {
                            r.has_ver_sw = true;
                            r.ver_sw = std::string(val);
                        } else if (key == "uint32_t ver_sw_release" && val.size() == 4) {
                            r.has_ver_sw_release = true;
                            r.ver_sw_release = rd32(val, 0);
                        }
                    } else if (val.size() == 4 && (key.substr(0, 6) == "float " || key.substr(0, 8) == "int32_t ")) {
                        UlogResult::Param p;
                        p.is_float = key[0] == 'f';
                        if (p.is_float) {
                            const uint32_t bits = rd32(val, 0);
                            std::memcpy(&p.f, &bits, 4);
                        } else {
                            p.i = static_cast<int32_t>(rd32(val, 0));
                        }
                        r.params[std::string(key.substr(key.find(' ') + 1))] = p;
                    }
                }
                break;
            case 'M':
                if (size < 2 || m[1] == 0 || static_cast<size_t>(static_cast<uint8_t>(m[1])) + 2 > size)
                    return bad("an 'M' message whose key length is zero or runs past the message", pos);
                break;
            case 'Q':
                if (size < 2 || (static_cast<uint8_t>(m[0]) & 3) == 0 || m[1] == 0 || static_cast<size_t>(static_cast<uint8_t>(m[1])) + 2 > size)
                    return bad("a 'Q' message with no default type bit, or a key length that is zero or runs past the message", pos);
                break;
            case 'A': {
                if (size < 4) return bad("a subscription of " + std::to_string(size) + " bytes, under 4", pos);
                const uint16_t id = rd16(m, 1);
                if (formats.count(std::string(m.substr(3))) == 0) return bad("a subscription (message id " + std::to_string(id) + ") to a format the file did not define", pos);
                if (subscribed.count(id) != 0) return bad("message id " + std::to_string(id) + " is subscribed twice", pos);
                subscribed[id] = true;
                subscribed_name[id] = std::string(m.substr(3));
                subscribed_multi[id] = static_cast<uint8_t>(m[0]);
                ++r.subscriptions;
                if (m[0] == 0) {
                    const std::string fname(m.substr(3));
                    const bool is_status = fname == kEkfStatusTopic;
                    const bool is_ratios = fname == kEkfRatiosTopic;
                    const bool is_flags = fname == kFailsafeTopic;
                    if (is_status || is_ratios || is_flags) {
                        Wanted w;
                        w.kind = is_status ? Wanted::Kind::status : (is_ratios ? Wanted::Kind::ratios : Wanted::Kind::flags);
                        std::string why;
                        const bool laid_out = parse_layout(format_text[fname], &w.fields, &why);
                        if (laid_out && is_status) {
                            const FieldLayout* fault = field_named(w.fields, "filter_fault_flags");
                            const FieldLayout* solution = field_named(w.fields, "solution_status_flags");
                            if (fault == nullptr || fault->type != "uint32_t" || fault->count != 1) {
                                why = "the format has no uint32_t filter_fault_flags field";
                            } else if (solution == nullptr || solution->type != "uint16_t" || solution->count != 1) {
                                why = "the format has no uint16_t solution_status_flags field";
                            } else {
                                w.fault_at = fault->offset;
                                w.solution_at = solution->offset;
                            }
                        } else if (laid_out) {
                            const std::string_view want_type = is_ratios ? "float" : "bool";
                            std::vector<std::string>& names = is_ratios ? r.test_ratios.names : r.failsafe.names;
                            for (size_t i = 0; i < w.fields.size(); ++i) {
                                const FieldLayout& f = w.fields[i];
                                if (f.type != want_type || f.name.rfind("_padding", 0) == 0) continue;
                                w.picked.push_back(i);
                                for (uint32_t k = 0; k < f.count; ++k) names.push_back(f.count == 1 ? f.name : f.name + "[" + std::to_string(k) + "]");
                            }
                            if (w.picked.empty()) why = std::string("the format has no ") + std::string(want_type) + " field";
                        }
                        std::string& problem = is_status ? r.ekf_status.problem : (is_ratios ? r.test_ratios.problem : r.failsafe.problem);
                        bool& present = is_status ? r.ekf_status.present : (is_ratios ? r.test_ratios.present : r.failsafe.present);
                        present = true;
                        if (!why.empty()) problem = why;
                        wanted[id] = std::move(w);
                    }
                }
                break;
            }
            case 'R':
                if (size != 2) return bad("an 'R' message of " + std::to_string(size) + " bytes, not 2", pos);
                break;
            case 'D': {
                if (size < 2) return bad("a data message of " + std::to_string(size) + " bytes, under 2", pos);
                const uint16_t id = rd16(m, 0);
                if (subscribed.count(id) == 0) return bad("a data message for message id " + std::to_string(id) + ", which was not subscribed", pos);
                ++r.data_messages;
                const std::string& fname = subscribed_name[id];
                if (!timed[fname]) {
                    ++r.untimed_messages;
                    if (r.untimed_formats.find(fname) == std::string::npos) r.untimed_formats += (r.untimed_formats.empty() ? "" : ", ") + fname;
                    break;
                }
                if (size < 10) return bad("a data message of " + std::to_string(size) + " bytes for a format that starts with a 64-bit timestamp, under 10", pos);
                const uint64_t ts = rd64(m, 2);
                ++r.timed_messages;
                if (ts != 0) {
                    const auto w = wanted.find(id);
                    if (w != wanted.end()) {
                        const std::string_view body = m.substr(2);  // the data after the message id: the timestamp is at offset 0
                        const auto too_short = [&](std::string& problem) {
                            if (problem.empty()) problem = "a data message is shorter than the fields the reader needs";
                        };
                        switch (w->second.kind) {
                            case Wanted::Kind::status: {
                                UlogResult::EkfStatusSeries& series = r.ekf_status;
                                if (!series.problem.empty()) break;
                                if (w->second.fault_at + 4 > body.size() || w->second.solution_at + 2 > body.size()) {
                                    too_short(series.problem);
                                    break;
                                }
                                UlogResult::EkfStatusSample sample;
                                sample.t_us = ts;
                                sample.fault_flags = static_cast<uint32_t>(rd_uint(body, w->second.fault_at, 4));
                                sample.solution_flags = static_cast<uint16_t>(rd_uint(body, w->second.solution_at, 2));
                                series.samples.push_back(sample);
                                break;
                            }
                            case Wanted::Kind::ratios: {
                                UlogResult::RatioSeries& series = r.test_ratios;
                                if (!series.problem.empty()) break;
                                std::vector<float> row;
                                bool short_body = false;
                                for (const size_t i : w->second.picked) {
                                    const FieldLayout& f = w->second.fields[i];
                                    for (uint32_t k = 0; k < f.count; ++k) {
                                        const size_t at = f.offset + k * f.size;
                                        if (at + 4 > body.size()) {
                                            short_body = true;
                                            break;
                                        }
                                        const uint32_t bits = rd32(body, at);
                                        float value = 0.0F;
                                        std::memcpy(&value, &bits, 4);
                                        row.push_back(value);
                                    }
                                    if (short_body) break;
                                }
                                if (short_body) {
                                    too_short(series.problem);
                                    break;
                                }
                                series.t_us.push_back(ts);
                                series.rows.push_back(std::move(row));
                                break;
                            }
                            case Wanted::Kind::flags: {
                                UlogResult::FlagSeries& series = r.failsafe;
                                if (!series.problem.empty()) break;
                                std::vector<uint8_t> row;
                                bool short_body = false;
                                for (const size_t i : w->second.picked) {
                                    const FieldLayout& f = w->second.fields[i];
                                    for (uint32_t k = 0; k < f.count; ++k) {
                                        const size_t at = f.offset + k;
                                        if (at + 1 > body.size()) {
                                            short_body = true;
                                            break;
                                        }
                                        row.push_back(body[at] != 0 ? 1 : 0);
                                    }
                                    if (short_body) break;
                                }
                                if (short_body) {
                                    too_short(series.problem);
                                    break;
                                }
                                series.t_us.push_back(ts);
                                series.rows.push_back(std::move(row));
                                break;
                            }
                        }
                    }
                }
                if (ts == 0) {
                    ++r.zero_stamps;
                    ++zero_by_format[fname];
                } else {
                    if (!r.has_time || ts < r.first_stamp_us) r.first_stamp_us = ts;
                    if (!r.has_time || ts > r.last_stamp_us) r.last_stamp_us = ts;
                    r.has_time = true;
                    UlogResult::Stream& st = stream_of[id];
                    if (st.n == 0) {
                        st.name = fname;
                        st.multi_id = subscribed_multi[id];
                        st.first_us = ts;
                    } else if (ts < st.last_us) {
                        st.monotone = false;
                    } else {
                        const uint64_t gap = ts - st.last_us;
                        st.first_at_gap.emplace(gap, st.last_us);
                        if (gap == 0) ++st.repeats;
                        st.gcd_us = std::gcd(st.gcd_us, gap);
                        st.min_gap_us = std::min(st.min_gap_us, gap);
                        st.max_gap_us = std::max(st.max_gap_us, gap);
                    }
                    if (st.n == 0 || ts >= st.last_us) st.last_us = ts;
                    ++st.n;
                }
                break;
            }
            case 'L':
                if (size < 9) return bad("a logged string of " + std::to_string(size) + " bytes, under 9", pos);
                r.log_lines.push_back({static_cast<uint8_t>(m[0]), rd64(m, 1), size - 9});
                break;
            case 'C':
                if (size < 11) return bad("a tagged logged string of " + std::to_string(size) + " bytes, under 11", pos);
                r.log_lines.push_back({static_cast<uint8_t>(m[0]), rd64(m, 3), size - 11});
                break;
            case 'S': {
                bool magic = size == 8;
                for (size_t i = 0; magic && i < 8; ++i) magic = static_cast<unsigned char>(m[i]) == kSync[i];
                if (!magic) return bad("a synchronization message whose body is not the 8 magic bytes", pos);
                ++r.syncs;
                break;
            }
            case 'O':
                if (size != 2) return bad("a dropout message of " + std::to_string(size) + " bytes, not 2", pos);
                ++r.dropouts;
                break;
            default: break;
        }
        first = false;
        pos = body_at + size;
    }
    if (first) return bad("the file holds no messages after the header", 16);
    if (r.data_messages == 0) return bad("the log holds no data message", b.size());
    for (auto& [id, st] : stream_of) r.streams.push_back(st);
    for (const auto& [name, n] : zero_by_format) r.zero_formats += (r.zero_formats.empty() ? "" : ", ") + name + " " + std::to_string(n);
    if (r.unknown_messages > 0) {
        r.kind = UlogResult::Kind::unread;
        r.detail = "unread: unknown message type " + r.unknown_types + " (" + std::to_string(r.unknown_messages) +
                   " messages, skipped with their framing intact); the rest of the log is well-formed, but a type this reader does not know is not read";
        return r;
    }
    r.kind = UlogResult::Kind::well_formed;
    r.detail = "well-formed: " + std::to_string(r.messages) + " messages (" + std::to_string(r.definitions) + " in the definitions, " + std::to_string(r.data_messages) +
               " logged data on " + std::to_string(r.subscriptions) + " subscriptions, " + std::to_string(r.syncs) + " syncs, " + std::to_string(r.dropouts) + " dropouts)";
    return r;
}

}  // namespace spade::fc

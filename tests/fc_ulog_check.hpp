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

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <string_view>

namespace spade::fc {

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
};

namespace ulog_detail {

[[nodiscard]] inline uint16_t rd16(std::string_view b, size_t at) {
    return static_cast<uint16_t>(static_cast<uint8_t>(b[at]) | (static_cast<uint16_t>(static_cast<uint8_t>(b[at + 1])) << 8));
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
                break;
            case 'I':
            case 'P':
                if (size < 1 || m[0] == 0 || static_cast<size_t>(static_cast<uint8_t>(m[0])) + 1 > size)
                    return bad(std::string("an ") + type + " message whose key length is zero or runs past the message", pos);
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
                ++r.subscriptions;
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
                break;
            }
            case 'L':
                if (size < 9) return bad("a logged string of " + std::to_string(size) + " bytes, under 9", pos);
                break;
            case 'C':
                if (size < 11) return bad("a tagged logged string of " + std::to_string(size) + " bytes, under 11", pos);
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

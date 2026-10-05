// The live smoke's verdict machinery, display-free (SL15b: headless is the
// test surface, so everything that decides something lives here and is
// asserted by tests/test_sandbox_live_smoke.cpp).
//
// The live smoke is a scripted tour of spade_sandbox through the real window,
// recorded to an MP4 so a person can watch what the engine does now
// (scripts/live-smoke.ps1). A smoke that cannot fail is a screenshot tool with
// opinions, so it keeps Kat's three mechanisms (KAT
// design-specs/dev/01-e2e-live-smoke.md, section 2.1):
//   1. anomalies, not assertions: a failed check is logged and the tour goes
//      on, and any anomaly fails the run;
//   2. known-open entries expire: a tolerated failure that stops firing fails
//      the run, so it cannot outlive its cause;
//   3. a declared-step ledger: a declared step that never ran fails the run,
//      and so does a step that ran but checked nothing.
// And Kat's section 4.4: the verdict is JOURNALED, not accumulated. Every event
// is written as a line the moment it happens, and the journal ends with an
// explicit "END verdict=..." line, so a crash leaves a true prefix whose
// missing terminator says the run did not finish.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "core/error.hpp"

namespace spade::sandbox::live {

// A tolerated anomaly, with why it is tolerated. It must fire on every run
// that lists it: one that does not is EXPIRED and fails the run.
struct KnownOpen {
    std::string id;
    std::string reason;
};

struct Verdict {
    bool pass = false;
    int anomalies = 0;         // not covered by a known-open entry: each fails the run
    int known_open_fired = 0;  // tolerated
    std::vector<std::string> unmarked;  // declared steps that never reached done()
    std::vector<std::string> expired;   // known-open ids that did not fire
};

// The ledger and the journal of one run. Steps are named "<main>.<sub>";
// a check inside a step gets the id "<main>.<sub>/<name>", and an anomaly
// outside any step names itself ("harness/...").
class Smoke {
  public:
    using LineWriter = std::function<void(std::string_view line)>;

    Smoke(std::vector<std::string> declared, std::vector<KnownOpen> known_open, LineWriter write)
        : declared_(std::move(declared)),
          marked_(declared_.size(), false),
          known_(std::move(known_open)),
          fired_(known_.size(), 0),
          write_(std::move(write)) {
        line("JOURNAL spade_sandbox live smoke v1");
        for (const std::string& s : declared_) {
            line("DECLARED " + s);
        }
        for (const KnownOpen& k : known_) {
            line("KNOWN-OPEN-LISTED " + k.id + " | " + k.reason);
        }
    }

    void note(std::string_view text) { line("NOTE " + std::string(text)); }

    // Starts a step. An undeclared step is an anomaly, so the ledger cannot
    // drift from the tour; starting one while another is open is too.
    void begin(std::string_view step) {
        if (!current_.empty()) {
            anomaly("harness/step-not-closed", current_);
        }
        current_ = std::string(step);
        checks_ = 0;
        line("BEGIN " + current_);
        if (index_of(current_) == kNone) {
            anomaly("harness/undeclared-step", current_);
        }
    }

    // Records a check in the current step. A failure is an anomaly and the run
    // goes on. Returns `ok`, so a caller can skip what depends on it.
    bool check(bool ok, std::string_view name, std::string_view detail) {
        const std::string id = (current_.empty() ? std::string("harness") : current_) + "/" + std::string(name);
        ++checks_;
        if (ok) {
            line("CHECK " + id + " | " + std::string(detail));
        } else {
            anomaly(id, detail);
        }
        return ok;
    }

    void anomaly(std::string_view id, std::string_view detail) {
        if (const std::size_t k = known_index(id); k != kNone) {
            ++fired_[k];
            line("KNOWN-OPEN " + std::string(id) + " | " + std::string(detail));
            return;
        }
        ++anomalies_;
        line("ANOMALY " + std::string(id) + " | " + std::string(detail));
    }

    // Ends the current step and marks it in the ledger. A step that checked
    // nothing proves nothing, so ending one is an anomaly.
    void done() {
        if (current_.empty()) {
            anomaly("harness/done-without-begin", "");
            return;
        }
        if (checks_ == 0) {
            anomaly(current_ + "/no-checks", "the step checked nothing");
        }
        line("DONE " + current_);
        if (const std::size_t i = index_of(current_); i != kNone) {
            marked_[i] = true;
        }
        current_.clear();
    }

    // The verdict so far. Writes nothing, so a closing card can show the
    // verdict the journal is about to record.
    [[nodiscard]] Verdict verdict() const {
        Verdict v;
        v.anomalies = anomalies_;
        for (std::size_t i = 0; i < declared_.size(); ++i) {
            if (!marked_[i]) {
                v.unmarked.push_back(declared_[i]);
            }
        }
        for (std::size_t k = 0; k < known_.size(); ++k) {
            if (fired_[k] == 0) {
                v.expired.push_back(known_[k].id);
            } else {
                v.known_open_fired += fired_[k];
            }
        }
        v.pass = v.anomalies == 0 && v.unmarked.empty() && v.expired.empty();
        return v;
    }

    // Writes the result and the terminator. Nothing may be journaled after it.
    Verdict finish() {
        const Verdict v = verdict();
        line("RESULT anomalies=" + std::to_string(v.anomalies) + " known_open_fired=" +
             std::to_string(v.known_open_fired) + " unmarked=" + std::to_string(v.unmarked.size()) +
             " expired=" + std::to_string(v.expired.size()));
        for (const std::string& u : v.unmarked) {
            line("UNMARKED " + u);
        }
        for (const std::string& e : v.expired) {
            line("EXPIRED " + e + " | " + known_[known_index(e)].reason);
        }
        line(std::string("END verdict=") + (v.pass ? "PASS" : "FAIL"));
        finished_ = true;
        return v;
    }

    [[nodiscard]] bool finished() const noexcept { return finished_; }
    [[nodiscard]] const std::string& current_step() const noexcept { return current_; }

  private:
    static constexpr std::size_t kNone = static_cast<std::size_t>(-1);

    void line(const std::string& text) {
        if (write_) {
            write_(text);
        }
    }
    [[nodiscard]] std::size_t index_of(std::string_view step) const {
        for (std::size_t i = 0; i < declared_.size(); ++i) {
            if (declared_[i] == step) return i;
        }
        return kNone;
    }
    [[nodiscard]] std::size_t known_index(std::string_view id) const {
        for (std::size_t k = 0; k < known_.size(); ++k) {
            if (known_[k].id == id) return k;
        }
        return kNone;
    }

    std::vector<std::string> declared_;
    std::vector<bool> marked_;
    std::vector<KnownOpen> known_;
    std::vector<int> fired_;
    LineWriter write_;
    std::string current_;
    int checks_ = 0;  // in the current step, passed or failed
    int anomalies_ = 0;
    bool finished_ = false;
};

// True when the journal's last line is its terminator. A journal without one
// is a run that did not finish, however complete it reads.
[[nodiscard]] inline bool is_terminated(std::string_view journal) {
    while (!journal.empty() && (journal.back() == '\n' || journal.back() == '\r')) {
        journal.remove_suffix(1);
    }
    const std::size_t start = journal.rfind('\n');
    const std::string_view last = start == std::string_view::npos ? journal : journal.substr(start + 1);
    return last.rfind("END verdict=", 0) == 0;
}

// ---------------------------------------------------------------------------
// The inject hooks, for the red runs: skip a declared step, or raise an
// anomaly inside one. An injection can only make a run red. There is no
// known-open injection, because a tolerated entry added from the command line
// could cancel a real anomaly and turn a FAIL into a PASS; expiry is covered
// by tests/test_sandbox_live_smoke.cpp.
// ---------------------------------------------------------------------------
struct Injection {
    enum class Kind { skip, anomaly };
    Kind kind = Kind::skip;
    std::string target;
};

[[nodiscard]] inline Result<Injection> parse_inject(std::string_view spec) {
    const auto refuse = [&]() -> Result<Injection> {
        return std::unexpected(Error{Code::invalid_argument,
                                     "--inject takes skip:<step> or anomaly:<step>/<name>, not '" +
                                         std::string(spec) + "'"});
    };
    const std::size_t colon = spec.find(':');
    if (colon == std::string_view::npos || colon + 1 >= spec.size()) {
        return refuse();
    }
    const std::string_view kind = spec.substr(0, colon);
    Injection out;
    out.target = std::string(spec.substr(colon + 1));
    if (kind == "skip") {
        out.kind = Injection::Kind::skip;
    } else if (kind == "anomaly") {
        out.kind = Injection::Kind::anomaly;
    } else {
        return refuse();
    }
    return out;
}

// Refuses, before the run starts, an injection that could not make it red: a
// skip of a step the ledger does not declare, an anomaly that is not
// "<declared step>/<name>", or an anomaly a known-open entry would tolerate.
[[nodiscard]] inline Result<void> validate_injections(std::span<const Injection> injections,
                                                      std::span<const std::string> declared,
                                                      std::span<const KnownOpen> known_open) {
    const auto is_declared = [&](std::string_view step) {
        for (const std::string& d : declared) {
            if (d == step) return true;
        }
        return false;
    };
    const auto steps = [&] {
        std::string list;
        for (const std::string& d : declared) {
            list += (list.empty() ? "" : ", ") + d;
        }
        return list;
    };
    for (const Injection& inj : injections) {
        if (inj.kind == Injection::Kind::skip) {
            if (!is_declared(inj.target)) {
                return std::unexpected(Error{Code::invalid_argument,
                                             "--inject skip:" + inj.target + " names no declared step, so it "
                                             "would skip nothing (the steps are " + steps() + ")"});
            }
            continue;
        }
        const std::size_t slash = inj.target.find('/');
        if (slash == std::string::npos || slash + 1 >= inj.target.size() ||
            !is_declared(std::string_view(inj.target).substr(0, slash))) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "--inject anomaly:" + inj.target + " must be <step>/<name> for a declared "
                                         "step (the steps are " + steps() + ")"});
        }
        for (const KnownOpen& k : known_open) {
            if (k.id == inj.target) {
                return std::unexpected(Error{Code::invalid_argument,
                                             "--inject anomaly:" + inj.target + " is a known-open entry, so it "
                                             "would be tolerated and the run would not go red"});
            }
        }
    }
    return {};
}

// ---------------------------------------------------------------------------
// The output folder. A run deletes exactly the names it writes, so a stale
// frame never poses as a new one (Kat's lesson). A folder holding anything
// else is refused before anything is deleted, because --out may name a folder
// that is not the run's to clear.
// ---------------------------------------------------------------------------

// Without ffmpeg a still is kept twice a second, up to this many (five
// minutes); the cap keeps the list of names a run owns finite and exact.
inline constexpr uint32_t kMaxStills = 600;

[[nodiscard]] inline std::string poster_name(uint32_t index, std::string_view main) {
    char prefix[8];
    std::snprintf(prefix, sizeof prefix, "%02u_", static_cast<unsigned>(index % 100u));
    return std::string(prefix) + std::string(main) + ".png";
}

[[nodiscard]] inline std::string still_name(uint32_t n) {
    char text[24];
    std::snprintf(text, sizeof text, "still_%05u.png", static_cast<unsigned>(n % 100000u));
    return text;
}

// Every name a run can write into its folder: the video, the journal, a poster
// per main function (numbered from 1, in tour order) and the stills.
[[nodiscard]] inline std::vector<std::string> owned_names(std::span<const std::string_view> mains) {
    std::vector<std::string> names = {"tour.mp4", "journal.txt"};
    for (std::size_t i = 0; i < mains.size(); ++i) {
        names.push_back(poster_name(static_cast<uint32_t>(i + 1), mains[i]));
    }
    for (uint32_t n = 0; n < kMaxStills; ++n) {
        names.push_back(still_name(n));
    }
    return names;
}

// What to delete before a run, given the folder's entries (a directory's name
// ends in '/'): every entry, when each is a name the run owns; otherwise
// nothing, and the first entry the run would not have written.
[[nodiscard]] inline Result<std::vector<std::string>> plan_sweep(const std::vector<std::string>& entries,
                                                                 std::span<const std::string> owned) {
    for (const std::string& e : entries) {
        bool ours = false;
        for (const std::string& o : owned) {
            if (o == e) {
                ours = true;
                break;
            }
        }
        if (!ours) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "holds '" + e + "', which a live smoke run does not write. A run deletes "
                                         "only its own files, so give --out an empty folder or an earlier run's"});
        }
    }
    return std::vector<std::string>(entries.begin(), entries.end());
}

// ---------------------------------------------------------------------------
// The recording and its title cards
// ---------------------------------------------------------------------------

// Raw RGBA frames on stdin, as glReadPixels returns them (bottom row first, so
// vflip), cropped to even dimensions (yuv420p needs them), into H.264. The
// filter is quoted: popen() hands the line to sh, where its parentheses are
// syntax, and an unquoted filter fails only at the first frame's write.
[[nodiscard]] inline std::string ffmpeg_command(const std::string& ffmpeg, const std::filesystem::path& out_mp4,
                                                uint32_t width, uint32_t height, uint32_t fps) {
    return ffmpeg + " -hide_banner -loglevel error -y -f rawvideo -pix_fmt rgba -s " + std::to_string(width) + "x" +
           std::to_string(height) + " -r " + std::to_string(fps) +
           " -i - -vf \"vflip,crop=trunc(iw/2)*2:trunc(ih/2)*2\" -c:v libx264 -preset veryfast -crf 20"
           " -pix_fmt yuv420p \"" +
           out_mp4.string() + "\"";
}

// The first seconds of the video, so a forwarded MP4 says what it shows.
// `label` is the commit and the date, from scripts/live-smoke.ps1.
[[nodiscard]] inline std::string opening_card(std::string_view label) {
    return "spade sandbox -- live smoke\n" + std::string(label) + "\nverdict at the end";
}

// The last seconds: the verdict and its counts.
[[nodiscard]] inline std::string closing_card(std::string_view label, const Verdict& v) {
    return "spade sandbox -- live smoke\n" + std::string(label) + "\nverdict " + (v.pass ? "PASS" : "FAIL") +
           "\nanomalies " + std::to_string(v.anomalies) + "   known-open " + std::to_string(v.known_open_fired) +
           "   unmarked " + std::to_string(v.unmarked.size()) + "   expired " + std::to_string(v.expired.size());
}

}  // namespace spade::sandbox::live

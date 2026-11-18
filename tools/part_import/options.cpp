#include "options.hpp"

#include <algorithm>

namespace spade::partimport {

namespace {

[[nodiscard]] Problem usage(const std::string& why) { return Problem{"usage", why}; }

}  // namespace

Expected<Options> parse_options(const std::vector<std::string>& args) {
    Options o;
    const auto set_mode = [&](Mode m, const std::string& flag) -> std::optional<Problem> {
        if (o.mode != Mode::none) return usage(flag + ": only one of --probe, --inspect and --import is allowed in a run");
        o.mode = m;
        return std::nullopt;
    };
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        const auto value = [&](std::optional<std::string>& slot) -> std::optional<Problem> {
            if (i + 1 >= args.size()) return usage(a + " needs a value");
            if (slot) return usage(a + " is given twice");
            slot = args[++i];
            return std::nullopt;
        };
        std::optional<Problem> p;
        if (a == "--help" || a == "-h") {
            o.help = true;
        } else if (a == "--probe") {
            p = set_mode(Mode::probe, a);
        } else if (a == "--inspect" || a == "--import") {
            p = set_mode(a == "--inspect" ? Mode::inspect : Mode::import, a);
            if (!p) {
                if (i + 1 >= args.size()) return std::unexpected(usage(a + " needs a file"));
                o.file = args[++i];
            }
        } else if (a == "--deep") {
            o.deep = true;
        } else if (a == "--store") {
            p = value(o.store);
        } else if (a == "--unit") {
            p = value(o.unit);
        } else if (a == "--axes") {
            p = value(o.axes);
        } else if (a == "--scale") {
            p = value(o.scale);
        } else if (a == "--object") {
            if (i + 1 >= args.size()) {
                p = usage(a + " needs a value");
            } else {
                const std::string& name = args[++i];
                if (std::find(o.objects.begin(), o.objects.end(), name) == o.objects.end()) o.objects.push_back(name);
            }
        } else if (a == "--deflection") {
            p = value(o.deflection);
        } else if (a == "--angle") {
            p = value(o.angle);
        } else if (a == "--step-python") {
            p = value(o.step_python);
        } else if (a == "--step-convert") {
            p = value(o.step_convert);
        } else if (!a.empty() && a.front() == '-') {
            return std::unexpected(usage("unknown option " + a));
        } else {
            return std::unexpected(usage("unexpected argument '" + a + "'"));
        }
        if (p) return std::unexpected(*p);
    }
    if (o.mode == Mode::none && !o.help) return std::unexpected(usage("no mode: give one of --probe, --inspect <file> or --import <file>"));
    return o;
}

const char* usage_text() {
    return "usage: spade_part_import --probe\n"
           "       spade_part_import --inspect <file> [--unit U] [--axes SPEC] [--scale S] [--object NAME]... [--deep --deflection LEN]\n"
           "       spade_part_import --import <file> --store <dir> [--unit U] [--axes SPEC] [--scale S] [--object NAME]...\n"
           "                         [--deflection LEN] [--angle RAD]\n"
           "\n"
           "  --probe            is the STEP converter usable (python, OpenCASCADE, versions), and how to install it when not\n"
           "  --inspect <file>   what the file holds, one gate result per object; writes nothing\n"
           "  --import <file>    the file and its sidecars into the part store (a cache hit when already there)\n"
           "  --store <dir>      the part store (default: $SPADE_PART_STORE); refused inside a git work tree\n"
           "  --unit U           m, mm, cm, um, in or ft: required for STL and OBJ on import; refused for the formats that declare theirs\n"
           "  --axes SPEC        three comma-separated source axes, each with an optional minus, for example -y,x,z: a rotation, not a mirror\n"
           "  --scale S          multiplies every position once, after the unit and the axes\n"
           "  --object NAME      one object of a file that holds several; repeat it to pick several (one entry, in any order)\n"
           "  STEP only: --deflection LEN (a length with a unit, no default), --angle RAD (default 0.5), --deep (inspect: tessellate\n"
           "             and run the gate), --step-python PATH, --step-convert PATH\n"
           "\n"
           "Standard output is JSON lines: stage events, then one result line last. Exit codes: 0 done, 1 io or internal, 2 usage,\n"
           "3 unavailable, 4 refused (not a good solid), 5 cancelled, 6 written but flagged.\n";
}

}  // namespace spade::partimport

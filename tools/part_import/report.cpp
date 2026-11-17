#include "report.hpp"

#include <cmath>
#include <format>

namespace spade::partimport {

namespace {

const std::vector<CodeInfo>& table() {
    static const std::vector<CodeInfo> codes = {
        {"ok", kExitOk, ""},

        {"store_io", kExitIo, "The part store could not be written. Check that the folder exists, has room and is not read-only."},
        {"file_unreadable", kExitIo, "The file could not be read. Check the path, and that no other program has it open."},
        {"internal", kExitIo, "An unexpected failure in the import tool. Run it again; if it repeats, report it with the result line."},

        {"usage", kExitUsage, "The command line is wrong; the message says which option."},
        {"unit_required", kExitUsage,
         "This format carries no unit. Give --unit (m, mm, cm, um, in or ft): the tool never guesses one."},
        {"unit_not_allowed", kExitUsage, "This format declares its own unit; leave --unit out."},
        {"axes_invalid", kExitUsage,
         "--axes takes three comma-separated axes, each x, y or z with an optional minus, each used once, for example -y,x,z."},
        {"axes_mirror", kExitUsage,
         "That axis choice is a mirror, which reverses the part. Change one sign, or swap two axes, so it is a rotation."},
        {"scale_invalid", kExitUsage, "--scale takes a number above zero, for example 0.001."},
        {"object_not_found", kExitUsage, "The file has no object of that name; the message lists the names it has."},
        {"deflection_required", kExitUsage,
         "A STEP file needs --deflection with a unit, for example 0.1mm: the tessellation error is a choice."},
        {"store_required", kExitUsage, "Give --store DIR, or set SPADE_PART_STORE."},
        {"store_in_git_tree", kExitUsage,
         "The store is inside a git work tree. Third-party CAD stays outside git: choose a folder elsewhere."},

        {"format_unknown", kExitUnavailable, "The file's type is not one this tool reads. Use STEP, 3MF, glTF or glb, STL or OBJ."},
        {"format_unavailable", kExitUnavailable,
         "The file uses something this build does not read; the message names it. Export it again in a plainer form (a glb without "
         "compression, a binary STL)."},
        {"external_buffers", kExitUnavailable, "The glTF refers to separate files. Export it as .glb, which holds everything in one file."},
        {"step_unavailable", kExitUnavailable,
         "STEP needs the OpenCASCADE converter, which is not installed here. spade_part_import --probe says how to install it."},
        {"converter_failed", kExitUnavailable, "The STEP converter stopped with an error; the message holds what it said."},

        {"file_invalid", kExitRefused, "The file is damaged or not what its type says; the message names the part. Export it again."},
        {"mesh_refused", kExitRefused, "The mesh was refused; the message says why. Export the part as a closed solid."},
        {"empty", kExitRefused, "The object has no triangles. Check that the export includes the body, not only a sketch."},
        {"bad_indices", kExitRefused, "The mesh's triangle list is damaged (a bad index, an unused point). Export it again."},
        {"non_finite", kExitRefused, "The mesh holds a position that is not a number. Export it again."},
        {"unit_invalid", kExitRefused, "The unit is not one this tool knows. Give --unit (m, mm, cm, um, in or ft)."},
        {"coincident_vertices", kExitRefused,
         "Two different points of the mesh are almost the same. Weld the vertices in the CAD, or export with a coarser tolerance."},
        {"degenerate_triangle", kExitRefused,
         "The mesh has a triangle with no area. Merge the vertices or remove the sliver in the CAD."},
        {"duplicate_triangle", kExitRefused, "The mesh has the same triangle twice. Remove the duplicate faces in the CAD."},
        {"not_closed", kExitRefused,
         "The mesh has open edges. Export the part as a closed solid (a watertight body), not a surface or a shell."},
        {"not_manifold", kExitRefused,
         "The mesh has an edge shared by three or more triangles, or a pinched point. Export a single clean solid."},
        {"inconsistent_winding", kExitRefused,
         "The mesh's faces do not all face outward. Recalculate the normals in the CAD, or export it again, so every face points out."},
        {"inside_out", kExitRefused, "The mesh is turned inside out. Flip its normals in the CAD, or export it again."},
        {"self_intersecting", kExitRefused,
         "The mesh passes through itself. Remove the overlapping parts in the CAD, or export the parts apart."},
        {"shells_not_nested", kExitRefused,
         "Two separate pieces of the mesh overlap without one holding the other. Export them as separate parts."},
        {"too_large", kExitRefused, "The mesh has more triangles than Spade holds. Export it coarser."},
        {"too_many_shells", kExitRefused, "The mesh has too many separate pieces. Export the pieces as separate parts."},

        {"cancelled", kExitCancelled, "The run was cancelled; nothing was stored."},
        {"cross_check_flagged", kExitFlagged,
         "The mesh was written, but its volume differs from the CAD's by more than 1%. Use a smaller --deflection or --angle."},
    };
    return codes;
}

}  // namespace

const std::vector<CodeInfo>& all_codes() { return table(); }

const CodeInfo& code_info(std::string_view code) {
    for (const CodeInfo& c : table()) {
        if (c.code == code) return c;
    }
    for (const CodeInfo& c : table()) {
        if (c.code == "internal") return c;
    }
    return table().front();  // not reached
}

std::string json_quote(std::string_view s) {
    std::string out = "\"";
    for (const char ch : s) {
        const auto c = static_cast<unsigned char>(ch);
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    out += std::format("\\u{:04x}", static_cast<unsigned>(c));
                } else {
                    out.push_back(ch);
                }
        }
    }
    out.push_back('"');
    return out;
}

std::string json_number(double v) { return std::isfinite(v) ? std::format("{}", v) : std::string("null"); }

namespace {
void put(std::string& body, std::string_view key, const std::string& value) {
    if (!body.empty()) body.push_back(',');
    body += json_quote(key);
    body.push_back(':');
    body += value;
}
}  // namespace

JsonObject& JsonObject::str(std::string_view key, std::string_view value) {
    put(body_, key, json_quote(value));
    return *this;
}
JsonObject& JsonObject::num(std::string_view key, double value) {
    put(body_, key, json_number(value));
    return *this;
}
JsonObject& JsonObject::integer(std::string_view key, std::int64_t value) {
    put(body_, key, std::to_string(value));
    return *this;
}
JsonObject& JsonObject::boolean(std::string_view key, bool value) {
    put(body_, key, value ? "true" : "false");
    return *this;
}
JsonObject& JsonObject::null(std::string_view key) {
    put(body_, key, "null");
    return *this;
}
JsonObject& JsonObject::raw(std::string_view key, std::string_view json) {
    put(body_, key, std::string(json));
    return *this;
}
std::string JsonObject::line() const { return "{" + body_ + "}"; }

}  // namespace spade::partimport

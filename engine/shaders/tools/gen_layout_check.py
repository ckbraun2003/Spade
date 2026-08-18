#!/usr/bin/env python3
"""Turn slangc reflection JSON into the D9 generated headers.

THE ONE GENERATION MECHANISM (engine design spec D9).  This script is the only
consumer of slangc's reflection output in the tree; cmake/SpadeSlang.cmake
invokes it exactly once per configure/build and emits both generated headers
from the single reflection record.  See that file's header comment for why
Python was chosen over CMake script mode.

INPUTS
  --reflection   slangc `-reflection-json` output for shaders/shared/bindings.slang
  --layouts      shaders/shared/layouts.slang   (source of the @cpp directives)
  --bindings     shaders/shared/bindings.slang  (source of the @set directives)

OUTPUTS (build tree only, never committed)
  --layout-out   layout_check.gen.hpp   -- pure static_asserts, one per field
                 of every mirrored struct plus per-struct size and alignment
  --bindings-out bindings.gen.hpp       -- namespace spade::compute::gen's
                 kSet_<name> / kBinding_<param> / count constants

WHAT MAKES THE BUILD FAIL, which is the whole point of the exercise:
  * a field reordered, resized, inserted or removed on EITHER side -- the
    generated static_assert for that field fails, and its message names the
    struct and the field;
  * a struct in layouts.slang carrying no @cpp directives (an unchecked
    mirror) -- this script exits non-zero;
  * an @cpp directive for a struct reflection never saw, i.e. a struct that no
    binding reaches and that therefore nothing checks -- this script exits
    non-zero;
  * a parameter bound into a descriptor set with no @set directive -- likewise.

WHAT THIS SCRIPT DELIBERATELY DOES NOT DO: decide anything.  Every number it
writes comes from the reflection record, and every name it writes comes from a
directive authored beside the declaration it describes.  There is no table of
struct names, offsets or binding indices anywhere in this file -- a table here
would be exactly the second source of truth D9 exists to abolish.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

# Structs reflection sees that carry no @cpp-type/@cpp-header directives and
# are still legal -- named here, once, with a reason, rather than being
# silently skipped. Every OTHER reflected struct must carry @cpp directives or
# this script fails.
#
# EMPTY AS OF S6 TASK 5. PassParams was the sole member from D9 (Task 3)
# through Task 4: it had no C++ counterpart because the CPU twin passes its
# values as function arguments, not a struct, so there was nothing to check it
# against. Task 5 gave it one (compute/vulkan/step_recorder.hpp's
# spade::compute::PassParams, the struct the host writes into the push-
# constant range) and added the matching @cpp directives directly above
# layouts.slang's `struct PassParams { ... }` -- once a struct carries those
# directives, having its name ALSO listed here is harmless (the "unclaimed"
# check below subtracts both directives and this set), but leaving a stale
# entry here would misdescribe why PassParams is exempt when it no longer is.
UNMIRRORED_STRUCTS = frozenset()

CPP_TYPE_RE = re.compile(r"^\s*//\s*@cpp-type\s+(\S+)\s*$")
CPP_HEADER_RE = re.compile(r"^\s*//\s*@cpp-header\s+(\S+)\s*$")
STRUCT_RE = re.compile(r"^\s*struct\s+([A-Za-z_]\w*)\s*\{")
SET_RE = re.compile(r"^\s*//\s*@set\s+(\d+)\s+([A-Za-z_]\w*)\s*$")


class GeneratorError(RuntimeError):
    pass


# ---------------------------------------------------------------------------
# Directive parsing
# ---------------------------------------------------------------------------


def parse_cpp_directives(path: Path) -> dict[str, tuple[str, str]]:
    """Slang struct name -> (fully qualified C++ type, include path).

    A directive pair binds to the NEXT `struct X {` line, so the two comments
    must sit immediately above the struct they describe (blank lines and other
    comment lines in between are tolerated; another struct in between is not).
    """
    mapping: dict[str, tuple[str, str]] = {}
    pending_type: str | None = None
    pending_header: str | None = None

    for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        m = CPP_TYPE_RE.match(line)
        if m:
            if pending_type is not None:
                raise GeneratorError(
                    f"{path}:{lineno}: two @cpp-type directives with no struct between them"
                )
            pending_type = m.group(1)
            continue

        m = CPP_HEADER_RE.match(line)
        if m:
            if pending_header is not None:
                raise GeneratorError(
                    f"{path}:{lineno}: two @cpp-header directives with no struct between them"
                )
            pending_header = m.group(1)
            continue

        m = STRUCT_RE.match(line)
        if m:
            name = m.group(1)
            if pending_type is None and pending_header is None:
                # No directives: legal only for the documented exemptions.
                if name not in UNMIRRORED_STRUCTS:
                    raise GeneratorError(
                        f"{path}:{lineno}: struct '{name}' has no @cpp-type/@cpp-header "
                        f"directives. Every mirrored struct must name the C++ type it "
                        f"mirrors, or be listed in UNMIRRORED_STRUCTS with a reason."
                    )
                continue
            if pending_type is None or pending_header is None:
                raise GeneratorError(
                    f"{path}:{lineno}: struct '{name}' has only one of @cpp-type/@cpp-header; "
                    f"both are required."
                )
            if name in mapping:
                raise GeneratorError(f"{path}:{lineno}: struct '{name}' declared twice")
            mapping[name] = (pending_type, pending_header)
            pending_type = None
            pending_header = None

    if pending_type is not None or pending_header is not None:
        raise GeneratorError(f"{path}: trailing @cpp directive with no struct after it")
    return mapping


def parse_set_directives(path: Path) -> dict[int, str]:
    """Descriptor set index -> authored set name."""
    sets: dict[int, str] = {}
    for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        m = SET_RE.match(line)
        if m:
            index = int(m.group(1))
            if index in sets:
                raise GeneratorError(f"{path}:{lineno}: descriptor set {index} named twice")
            sets[index] = m.group(2)
    if not sets:
        raise GeneratorError(f"{path}: no @set directives found")
    return sets


# ---------------------------------------------------------------------------
# Reflection walking
# ---------------------------------------------------------------------------


def uniform_size(node: dict) -> tuple[int, int]:
    """(size, alignment) from a reflection node's `sizes` list."""
    for entry in node.get("sizes", []):
        if entry.get("kind") == "uniform":
            return int(entry["value"]), int(entry["alignment"])
    raise GeneratorError(f"reflection node has no uniform size: {node.get('name', node)}")


def collect_structs(node: object, out: dict[str, dict]) -> None:
    """Depth-first walk of a reflection subtree, collecting every struct type.

    Recursive because a struct's fields may themselves be structs (RngStream
    inside DrydenState and ImuSensorRow; ContactParams and GridParams inside
    PassParams), and a nested struct's own field offsets are just as much part
    of the enclosing row's byte image as the enclosing struct's are.
    """
    if isinstance(node, list):
        for item in node:
            collect_structs(item, out)
        return
    if not isinstance(node, dict):
        return

    if node.get("kind") == "struct" and "name" in node and "fields" in node:
        name = node["name"]
        size, alignment = uniform_size(node)
        fields = []
        for field in node["fields"]:
            binding = field["binding"]
            if binding.get("kind") != "uniform":
                raise GeneratorError(
                    f"struct '{name}' field '{field['name']}' has non-uniform binding "
                    f"{binding!r}; a state row's fields must all be plain data."
                )
            fields.append(
                (field["name"], int(binding["offset"]), int(binding["size"]))
            )
        record = {"size": size, "alignment": alignment, "fields": fields}
        previous = out.get(name)
        if previous is not None and previous != record:
            raise GeneratorError(
                f"struct '{name}' reflected with two different layouts: "
                f"{previous!r} vs {record!r}"
            )
        out[name] = record

    for value in node.values():
        collect_structs(value, out)


def collect_bindings(reflection: dict) -> tuple[list[tuple[str, int, int]], list[tuple[str, int]]]:
    """(descriptor-table parameters, push-constant parameters).

    Descriptor entries are (name, set, index); push-constant entries are
    (name, byte size).  `space` is absent from the reflection record when it is
    zero, which is why it is read with a default rather than indexed.
    """
    descriptors: list[tuple[str, int, int]] = []
    push_constants: list[tuple[str, int]] = []

    for param in reflection.get("parameters", []):
        name = param["name"]
        binding = param["binding"]
        kind = binding.get("kind")
        if kind == "descriptorTableSlot":
            descriptors.append((name, int(binding.get("space", 0)), int(binding["index"])))
        elif kind == "pushConstantBuffer":
            element = param["type"].get("elementType")
            if element is None:
                raise GeneratorError(f"push constant '{name}' has no element type")
            size, _ = uniform_size(element)
            push_constants.append((name, size))
        else:
            raise GeneratorError(
                f"parameter '{name}' has unsupported binding kind '{kind}'. The D9 "
                f"registry knows storage buffers and one push-constant block; anything "
                f"else needs a deliberate decision, not a silent default."
            )
    return descriptors, push_constants


# ---------------------------------------------------------------------------
# Emission
# ---------------------------------------------------------------------------

BANNER = """// GENERATED FILE -- DO NOT EDIT, DO NOT COMMIT.
//
// Written by engine/shaders/tools/gen_layout_check.py from slangc reflection
// over engine/shaders/shared/{source}. It lives in the BUILD tree only
// (cmake/SpadeSlang.cmake decides where); editing it is pointless, because the
// next build overwrites it, and committing it would reintroduce exactly the
// hand-maintained duplicate D9 exists to abolish.
//
// To change anything here, change the Slang source it was generated from.
"""


def emit_layout_check(
    structs: dict[str, dict], directives: dict[str, tuple[str, str]]
) -> str:
    unclaimed = sorted(set(structs) - set(directives) - UNMIRRORED_STRUCTS)
    if unclaimed:
        raise GeneratorError(
            "reflected struct(s) with no @cpp directives: "
            + ", ".join(unclaimed)
            + ". Every mirrored struct must name the C++ type it mirrors."
        )
    unreflected = sorted(set(directives) - set(structs))
    if unreflected:
        raise GeneratorError(
            "struct(s) declared in layouts.slang that no binding reaches, so nothing "
            "reflects or checks them: "
            + ", ".join(unreflected)
            + ". Bind them in bindings.slang or remove them."
        )

    headers = sorted({header for _, header in directives.values()})

    out: list[str] = [BANNER.format(source="layouts.slang")]
    out.append("#pragma once")
    out.append("")
    out.append("#include <cstddef>")
    out.append("#include <cstdint>")
    out.append("")
    for header in headers:
        out.append(f'#include "{header}"')
    out.append("")
    out.append(
        "// Every assertion below compares a C++ struct against the byte offsets slangc\n"
        "// REPORTED for the corresponding Slang struct -- not against a number a human\n"
        "// typed twice. sizeof and every offset are asserted exactly; alignment is\n"
        "// asserted as divisibility, because several C++ counterparts are declared\n"
        "// alignas(16) where std430 only requires 4 or 8. Over-alignment changes no\n"
        "// offset and no size, and a buffer offset that satisfies 16 satisfies 4; UNDER-\n"
        "// alignment is what would break, and that is what the divisibility catches."
    )
    out.append("")

    for slang_name in sorted(directives):
        cpp_type, _ = directives[slang_name]
        record = structs[slang_name]
        out.append("// " + "-" * 74)
        out.append(f"// {slang_name} <-> {cpp_type}")
        out.append("// " + "-" * 74)
        out.append(
            f"static_assert(sizeof({cpp_type}) == {record['size']}u,\n"
            f'              "layouts.slang {slang_name}: sizeof drifted from the Slang '
            f'struct (slangc reports {record["size"]})");'
        )
        out.append(
            f"static_assert(alignof({cpp_type}) % {record['alignment']}u == 0u,\n"
            f'              "layouts.slang {slang_name}: C++ alignment does not satisfy the '
            f'Slang std430 alignment ({record["alignment"]})");'
        )
        for field_name, offset, size in record["fields"]:
            out.append(
                f"static_assert(offsetof({cpp_type}, {field_name}) == {offset}u,\n"
                f'              "layouts.slang {slang_name}.{field_name}: OFFSET drifted '
                f'(slangc reports {offset})");'
            )
            out.append(
                f"static_assert(sizeof({cpp_type}::{field_name}) == {size}u,\n"
                f'              "layouts.slang {slang_name}.{field_name}: SIZE drifted '
                f'(slangc reports {size})");'
            )
        out.append("")

    out.append(
        "// The mirrored-struct count, so a struct silently dropped from layouts.slang\n"
        "// (or from the binding registry that makes it reflectable) is visible to\n"
        "// tests/test_slang_layouts.cpp rather than merely producing fewer asserts."
    )
    out.append("namespace spade::compute::gen {")
    out.append(
        f"inline constexpr std::size_t kMirroredStructCount = {len(directives)};"
    )
    out.append("}  // namespace spade::compute::gen")
    out.append("")
    return "\n".join(out)


def emit_bindings(
    descriptors: list[tuple[str, int, int]],
    push_constants: list[tuple[str, int]],
    set_names: dict[int, str],
) -> str:
    unnamed = sorted({space for _, space, _ in descriptors} - set(set_names))
    if unnamed:
        raise GeneratorError(
            "descriptor set(s) with no @set directive in bindings.slang: "
            + ", ".join(str(s) for s in unnamed)
        )
    if len(push_constants) != 1:
        raise GeneratorError(
            f"expected exactly one push-constant block, found {len(push_constants)}: "
            f"{[name for name, _ in push_constants]}"
        )

    out: list[str] = [BANNER.format(source="bindings.slang")]
    out.append("#pragma once")
    out.append("")
    out.append("#include <cstdint>")
    out.append("")
    out.append("namespace spade::compute::gen {")
    out.append("")
    out.append("// Descriptor set numbers, named by bindings.slang's @set directives.")
    for space in sorted(set_names):
        out.append(f"inline constexpr uint32_t kSet_{set_names[space]} = {space}u;")
    out.append("")
    out.append(
        "// One constant per bound shader parameter, in binding order. The name is the\n"
        "// Slang parameter's own -- a registered array's `.slot_to_world` sibling is\n"
        "// spelled with an underscore because Slang identifiers cannot carry a dot."
    )
    for name, space, index in sorted(descriptors, key=lambda d: (d[1], d[2])):
        out.append(f"inline constexpr uint32_t kBinding_{name} = {index}u;")
    out.append("")
    for space in sorted(set_names):
        count = sum(1 for _, s, _ in descriptors if s == space)
        out.append(
            f"// Bindings in set {space} ({set_names[space]}). A descriptor-pool sizing\n"
            f"// input, and the number test_slang_layouts.cpp pins so nothing can be added\n"
            f"// to or dropped from the registry without a test moving."
        )
        out.append(
            f"inline constexpr uint32_t kBindingCount_{set_names[space]} = {count}u;"
        )
    out.append("")
    pc_name, pc_size = push_constants[0]
    out.append(
        f"// The single push-constant block ({pc_name}). Vulkan guarantees at least 128\n"
        f"// bytes on every device; this must stay under that."
    )
    out.append(f"inline constexpr uint32_t kPushConstantSize = {pc_size}u;")
    out.append("inline constexpr uint32_t kPushConstantOffset = 0u;")
    out.append("")
    out.append("}  // namespace spade::compute::gen")
    out.append("")
    return "\n".join(out)


# ---------------------------------------------------------------------------


def write_if_changed(path: Path, text: str) -> None:
    """Avoid touching an unchanged output so ninja does not rebuild the world."""
    if path.exists() and path.read_text(encoding="ascii") == text:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="ascii")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reflection", type=Path, required=True)
    parser.add_argument("--layouts", type=Path, required=True)
    parser.add_argument("--bindings", type=Path, required=True)
    parser.add_argument("--layout-out", type=Path, required=True)
    parser.add_argument("--bindings-out", type=Path, required=True)
    args = parser.parse_args(argv)

    try:
        reflection = json.loads(args.reflection.read_text(encoding="utf-8"))
        directives = parse_cpp_directives(args.layouts)
        set_names = parse_set_directives(args.bindings)

        structs: dict[str, dict] = {}
        collect_structs(reflection, structs)
        descriptors, push_constants = collect_bindings(reflection)

        write_if_changed(args.layout_out, emit_layout_check(structs, directives))
        write_if_changed(args.bindings_out, emit_bindings(descriptors, push_constants, set_names))
    except GeneratorError as exc:
        print(f"gen_layout_check.py: error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

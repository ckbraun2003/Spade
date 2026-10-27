#!/usr/bin/env python3
"""Embed a compiled SPIR-V module into a C++ header as a uint32_t array.

WHY EMBED AT ALL, rather than loading the .spv at runtime.  The kat testing
spec's no-CWD rule -- a binary must never depend on where it was launched from
to find its data -- applies to shaders exactly as it applies to the golden
corpus, and a shader is worse: a missing or stale .spv surfaces as a device
error deep inside a dispatch rather than as a file-not-found.  Embedding makes
the module part of the linked image, so a build that succeeded cannot be
missing its kernels, and there is no install-time shader directory to keep in
step with the binary.

WHY THIS IS A SECOND SCRIPT AND NOT A SECOND MECHANISM.  Its job -- turn bytes
into a C array -- has nothing to do with gen_layout_check.py's (consume slangc
reflection and emit checked declarations); they share no logic and neither
could reuse the other's code.  What the D9 brief forbids is TWO IMPLEMENTATIONS
OF THE SAME GENERATION STEP (a Python generator and a CMake-script generator
doing the same work).  Both scripts here are Python, invoked by the same
cmake/SpadeSlang.cmake with the same interpreter -- one mechanism, two
single-responsibility files.

WHY ONE HEADER CAN HOLD SEVERAL MODULES (S6 Task 9b).  A compute kernel's local
size is baked into its SPIR-V by `[numthreads(...)]`, so making
`BackendDesc::workgroup_size` a REAL knob means compiling the same kernel source
once per legal size.  This script therefore takes either ONE `--input` (a kernel
with no knob to serve -- fp32_math_probe) or several `--variant SIZE=PATH` pairs,
and in both cases emits a `kSpvVariants_<symbol>` table of
compute/spirv_variants.hpp's `SpirvVariantSet` type.  Consumers walk that table
rather than naming arrays one by one, which is what keeps
tests/test_slang_layouts.cpp's SPIR-V policy scan automatically covering every
compiled variant instead of a hand-maintained list of them.

WHY --nan-preserve (the airflow kernels, 2026-10-27). Without the
SignedZeroInfNanPreserve execution mode, Vulkan lets a driver assume no float is
NaN, and the RTX 3060 Ti's driver 572.83 does: it compiles advection's clamp, a
pair of ordered compares and branches in the SPIR-V, into a min/max that drops
a NaN where the CPU's carries it (Rendering's bisect, docs/design/backlog.md).
slangc v2026.14.1 has no flag for the mode, so this flag adds it to each module
before embedding: the capability, the SPV_KHR_float_controls extension, and
`OpExecutionMode <entry> SignedZeroInfNanPreserve 32` on every entry point. No
new ids, so the header's bound stands. A device that does not support the mode
for fp32 is refused by name when the pipeline is made
(compute/vulkan/dispatch_chain.hpp); rule P6 in engine/testing/spirv_scan.hpp
checks that exactly the airflow modules declare it.

USAGE
    embed_spirv.py --input <module>.spv --output <module>.spv.gen.hpp \
                   --symbol <module>

    embed_spirv.py --variant 32=<module>.wg32.spv \
                   --variant 64=<module>.wg64.spv \
                   --variant 128=<module>.wg128.spv \
                   --output <module>.spv.gen.hpp --symbol <module> \
                   [--nan-preserve]
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

SPIRV_MAGIC = 0x07230203

# What --nan-preserve writes (Khronos SPIRV-Headers, unified1/spirv.hpp).
OP_EXTENSION = 10
OP_ENTRY_POINT = 15
OP_EXECUTION_MODE = 16
OP_CAPABILITY = 17
CAPABILITY_SIGNED_ZERO_INF_NAN_PRESERVE = 4466
EXECUTION_MODE_SIGNED_ZERO_INF_NAN_PRESERVE = 4461
FLOAT_WIDTH_32 = 32
FLOAT_CONTROLS_EXTENSION = "SPV_KHR_float_controls"

BANNER = """// GENERATED FILE -- DO NOT EDIT, DO NOT COMMIT.
//
// Written by engine/shaders/tools/embed_spirv.py from the slangc-compiled
// SPIR-V for engine/shaders/kernels/{symbol}.slang. It lives in the BUILD tree
// only; the next build overwrites it.
"""


def read_module(path: Path) -> tuple[int, ...]:
    """Read one .spv and return its words, or raise ValueError with the reason."""
    blob = path.read_bytes()
    if len(blob) % 4 != 0:
        raise ValueError(
            f"{path} is {len(blob)} bytes, not a whole number of 32-bit words"
        )
    words = struct.unpack(f"<{len(blob) // 4}I", blob)
    if not words or words[0] != SPIRV_MAGIC:
        raise ValueError(
            f"{path} does not start with the SPIR-V magic number (0x{SPIRV_MAGIC:08x})"
        )
    return words


def instructions(words: tuple[int, ...]):
    """Yield (word index, opcode, word count) for each instruction past the header."""
    i = 5
    while i < len(words):
        count = words[i] >> 16
        if count == 0 or i + count > len(words):
            raise ValueError(f"truncated or zero-length instruction at word {i}")
        yield i, words[i] & 0xFFFF, count
        i += count


def literal_string(text: str) -> list[int]:
    """`text` as a SPIR-V literal: UTF-8, NUL-terminated, packed little-endian into words."""
    data = text.encode("utf-8") + b"\0"
    data += b"\0" * (-len(data) % 4)
    return list(struct.unpack(f"<{len(data) // 4}I", data))


def preserve_nan(words: tuple[int, ...]) -> tuple[int, ...]:
    """`words` declaring SignedZeroInfNanPreserve for fp32 (this file's --nan-preserve note).

    The capability goes after the module's last OpCapability, the extension
    after its last OpExtension (or the capabilities, when it has none), and the
    mode after its last OpEntryPoint, once per entry point: the order SPIR-V's
    logical layout requires. Whatever the module already declares is kept, not
    repeated."""
    entries: list[int] = []
    moded: list[int] = []
    has_capability = False
    has_extension = False
    last_capability = last_extension = last_entry = None
    for i, op, count in instructions(words):
        if op == OP_CAPABILITY:
            last_capability = i
            has_capability |= words[i + 1] == CAPABILITY_SIGNED_ZERO_INF_NAN_PRESERVE
        elif op == OP_EXTENSION:
            last_extension = i
            has_extension |= list(words[i + 1 : i + count]) == literal_string(FLOAT_CONTROLS_EXTENSION)
        elif op == OP_ENTRY_POINT:
            last_entry = i
            entries.append(words[i + 2])
        elif (op == OP_EXECUTION_MODE and count == 4 and
              words[i + 2] == EXECUTION_MODE_SIGNED_ZERO_INF_NAN_PRESERVE and words[i + 3] == FLOAT_WIDTH_32):
            moded.append(words[i + 1])
    if last_capability is None or last_entry is None:
        raise ValueError("--nan-preserve: the module has no OpCapability or no OpEntryPoint")

    after: dict[int, list[int]] = {}
    if not has_capability:
        after.setdefault(last_capability, []).extend(
            [(2 << 16) | OP_CAPABILITY, CAPABILITY_SIGNED_ZERO_INF_NAN_PRESERVE])
    if not has_extension:
        name = literal_string(FLOAT_CONTROLS_EXTENSION)
        anchor = last_capability if last_extension is None else last_extension
        after.setdefault(anchor, []).extend([((1 + len(name)) << 16) | OP_EXTENSION] + name)
    for entry in entries:
        if entry not in moded:
            after.setdefault(last_entry, []).extend(
                [(4 << 16) | OP_EXECUTION_MODE, entry, EXECUTION_MODE_SIGNED_ZERO_INF_NAN_PRESERVE, FLOAT_WIDTH_32])

    out = list(words[:5])
    for i, _, count in instructions(words):
        out.extend(words[i : i + count])
        out.extend(after.get(i, []))
    return tuple(out)


def parse_variant(spec: str) -> tuple[int, Path]:
    """Parse a `--variant SIZE=PATH` argument."""
    size_text, separator, path_text = spec.partition("=")
    if not separator or not path_text:
        raise ValueError(f"--variant expects SIZE=PATH, got {spec!r}")
    try:
        size = int(size_text)
    except ValueError:
        raise ValueError(f"--variant workgroup size {size_text!r} is not an integer")
    if size <= 0:
        raise ValueError(f"--variant workgroup size {size} is not positive")
    return size, Path(path_text)


def emit_array(lines: list[str], symbol: str, source: Path, words: tuple[int, ...],
               nan_preserve: bool) -> None:
    """Append one embedded uint32_t[] plus its word count."""
    as_emitted = ("as slangc emitted them, plus SignedZeroInfNanPreserve 32\n"
                  "// (embed_spirv.py --nan-preserve)" if nan_preserve else "exactly as slangc emitted them")
    lines.append(
        f"// {source.name}: {len(words)} words ({len(words) * 4} bytes), little-endian,\n"
        f"// {as_emitted}. vkCreateShaderModule takes this array\n"
        f"// verbatim -- no byte-swapping, no header stripping."
    )
    lines.append(f"inline constexpr uint32_t kSpv_{symbol}[] = {{")
    for start in range(0, len(words), 8):
        chunk = ", ".join(f"0x{word:08x}u" for word in words[start : start + 8])
        lines.append(f"    {chunk},")
    lines.append("};")
    lines.append("")
    lines.append(f"inline constexpr std::size_t kSpvWordCount_{symbol} = {len(words)};")
    lines.append("")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path)
    parser.add_argument(
        "--variant",
        action="append",
        default=[],
        metavar="SIZE=PATH",
        help="a per-workgroup-size compiled module; repeatable",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--symbol", required=True)
    parser.add_argument(
        "--nan-preserve",
        action="store_true",
        help="declare SignedZeroInfNanPreserve 32 on every entry point (the airflow kernels)",
    )
    args = parser.parse_args(argv)

    if (args.input is None) == (not args.variant):
        print(
            "embed_spirv.py: error: pass exactly one of --input (a single module) or "
            "one-or-more --variant SIZE=PATH (a per-workgroup-size set)",
            file=sys.stderr,
        )
        return 1

    # (workgroup_size, array symbol suffix, source path, words). The SINGLE-input
    # case is modelled as a one-entry set at 64 -- the literal `[numthreads(64,1,1)]`
    # such a kernel spells -- rather than as a separate shape, so every consumer
    # walks one table type and no module can be omitted from the SPIR-V policy
    # scan by being the odd one out.
    entries: list[tuple[int, str, Path, tuple[int, ...]]] = []
    read = (lambda path: preserve_nan(read_module(path))) if args.nan_preserve else read_module
    try:
        if args.input is not None:
            entries.append((64, args.symbol, args.input, read(args.input)))
        else:
            for spec in args.variant:
                size, path = parse_variant(spec)
                entries.append((size, f"{args.symbol}_wg{size}", path, read(path)))
    except ValueError as error:
        print(f"embed_spirv.py: error: {error}", file=sys.stderr)
        return 1

    sizes = [size for size, _, _, _ in entries]
    if len(set(sizes)) != len(sizes):
        print(
            f"embed_spirv.py: error: duplicate workgroup size in {sizes} for "
            f"symbol {args.symbol}",
            file=sys.stderr,
        )
        return 1

    lines: list[str] = [BANNER.format(symbol=args.symbol)]
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <cstddef>")
    lines.append("#include <cstdint>")
    lines.append("")
    lines.append('#include "compute/spirv_variants.hpp"')
    lines.append("")
    lines.append("namespace spade::compute::gen {")
    lines.append("")
    for _, array_symbol, source, words in entries:
        emit_array(lines, array_symbol, source, words, args.nan_preserve)

    lines.append(
        f"// Every compiled module for {args.symbol}.slang, keyed by the local size it\n"
        f"// was compiled for. compute/vulkan/step_recorder.cpp selects from this by\n"
        f"// BackendDesc::workgroup_size; tests/test_slang_layouts.cpp scans EVERY row."
    )
    lines.append(f"inline constexpr SpirvVariant kSpvVariantList_{args.symbol}[] = {{")
    for size, array_symbol, _, _ in entries:
        lines.append(
            f"    {{{size}u, kSpv_{array_symbol}, kSpvWordCount_{array_symbol}}},"
        )
    lines.append("};")
    lines.append("")
    lines.append(
        f"inline constexpr SpirvVariantSet kSpvVariants_{args.symbol}{{"
        f'"{args.symbol}", kSpvVariantList_{args.symbol}, {len(entries)}}};'
    )
    lines.append("")
    lines.append("}  // namespace spade::compute::gen")
    lines.append("")

    text = "\n".join(lines)
    if args.output.exists() and args.output.read_text(encoding="ascii") == text:
        return 0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text, encoding="ascii")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

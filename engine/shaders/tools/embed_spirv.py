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

USAGE
    embed_spirv.py --input <module>.spv --output <module>.spv.gen.hpp \
                   --symbol <module>

    embed_spirv.py --variant 32=<module>.wg32.spv \
                   --variant 64=<module>.wg64.spv \
                   --variant 128=<module>.wg128.spv \
                   --output <module>.spv.gen.hpp --symbol <module>
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

SPIRV_MAGIC = 0x07230203

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


def emit_array(lines: list[str], symbol: str, source: Path, words: tuple[int, ...]) -> None:
    """Append one embedded uint32_t[] plus its word count."""
    lines.append(
        f"// {source.name}: {len(words)} words ({len(words) * 4} bytes), little-endian,\n"
        f"// exactly as slangc emitted them. vkCreateShaderModule takes this array\n"
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
    try:
        if args.input is not None:
            entries.append((64, args.symbol, args.input, read_module(args.input)))
        else:
            for spec in args.variant:
                size, path = parse_variant(spec)
                entries.append((size, f"{args.symbol}_wg{size}", path, read_module(path)))
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
        emit_array(lines, array_symbol, source, words)

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

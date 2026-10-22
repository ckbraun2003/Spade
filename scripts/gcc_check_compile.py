"""scripts/gcc-check.sh --compile, inside the leg image: compile each named .cpp
in full with the leg volume's own command line (Release, -O3), so the
optimiser's warnings -- maybe-uninitialized, array-bounds and the like, which
-fsyntax-only never runs -- surface before a leg does.

    python3 gcc_check_compile.py <volume> <file.cpp>...

Run by gcc-check.sh with the volume at /leg (read-only) and the worktree's
sources copied to /tmp/s. For each file it takes the command ninja records for
that translation unit (`ninja -t compdb`), points its sources at /tmp/s (the
repository root's -I/leg/src too, which spike/ includes are spelled from),
puts the worktree build's generated-header directories ahead of the volume's
(GCC_CHECK_BUILD, from gcc-check.sh), drops the depfile flags (the volume is
read-only), writes the object to /tmp, and adds -Wall -Wextra -Wpedantic
-Werror. A file the volume's build never
compiled fails with the reason: run a leg first -- except a new spade_tests
file (NEW_TESTS below), which borrows a sibling's command line, and the output
says whose.

Window-backend code. The leg's container has no X11, so GLFW builds without a
window backend (SPADE_GLFW_HAS_BACKEND=0) and some code never meets gcc there:
- a target the leg never builds borrows the command line of a named file the
  leg does build, of the same family (TEMPLATES), and the output says so;
- a file the leg builds with its backend branch off is compiled a second time
  with it on (BACKEND_ON);
- a backend branch nothing here can compile is named in the output (UNCHECKED),
  so a pass is not read as covering it.
"""
import json
import os
import re
import shlex
import subprocess
import sys

# (path prefix, template file, why). The template has the same include
# directories and defines the prefix's files need: tests/test_gpu_gl_renderer.cpp
# builds against render_gl, glad and GLFW with GLFW_INCLUDE_NONE and
# SPADE_TESTS_DIR, as flow_movie does (Rendering's stand-in, made permanent by
# the lead, 2026-10-13).
TEMPLATES = [
    ("tools/flow_movie/", "tests/test_gpu_gl_renderer.cpp",
     "the leg never builds tools/flow_movie, which needs a window backend"),
    # Read only for a file the volume's build lacks: every spade_render source
    # is compiled with the target's one command line (no per-file flags).
    ("engine/render/", "engine/render/flow_lines.cpp",
     "a new spade_render source the leg has not built yet"),
    # Likewise spade_airflow_runner (spike/airflow/CMakeLists.txt sets no
    # per-file flags), plan B's runner and kernels' host side.
    ("spike/airflow/", "spike/airflow/chain_runner.cpp",
     "a new spade_airflow_runner source the leg has not built yet"),
]

# file -> (flags replaced, flags added): the leg's own line, with the window
# backend on. imgui and GLFW are fetched into every build's _deps, so their
# headers are there even where no target links them.
BACKEND_ON = {
    "sandbox/editor_panels_ui.cpp": (
        {"-DSPADE_SANDBOX_HAS_GL=0": "-DSPADE_SANDBOX_HAS_GL=1"},
        ["-DGLFW_INCLUDE_NONE", "-I/leg/build/_deps/imgui-src",
         "-I/leg/build/_deps/imgui-src/backends", "-I/leg/build/_deps/glfw-src/include"]),
}

UNCHECKED = {
    "sandbox/gl_target_sink.cpp":
        "its SPADE_SANDBOX_HAS_GL=1 branch includes <GL/gl.h>, which the leg's image lacks, so only MSVC compiles it",
}

# A new spade_tests file. Every tests/test_*.cpp is a spade_tests translation
# unit, and CMake gives each one the target's single command line (no file in
# tests/CMakeLists.txt sets flags of its own), so one the volume's build never
# compiled borrows a sibling's: the first spade_tests file, in path order, that
# the build does compile (the lead, 2026-10-19: a new test file was the one
# file --compile could not check without a leg).
NEW_TESTS = "tests/test_"


def is_new_test(f):
    return f.startswith(NEW_TESTS) and f.endswith(".cpp") and "/" not in f[len("tests/"):]


def spade_tests_sibling(entries):
    """(file, entry) of the first spade_tests translation unit the build compiles, or (None, None)."""
    for name in sorted(entries):
        entry = entries[name]
        line = entry.get("command", " ".join(entry.get("arguments", [])))
        if name.startswith("/leg/src/" + NEW_TESTS) and "/spade_tests.dir/" in entry.get("output", line):
            return name[len("/leg/src/"):], entry
    return None, None


DROP_ONE = {"-MD", "-MMD"}
DROP_TWO = {"-MF", "-MT", "-MQ", "-o"}

# The volume's sources: a path under /leg/src, or /leg/src itself (the
# repository root, -I/leg/src). Left as it was, the root would read the
# volume's spike/ headers, not the branch's.
LEG_SRC = re.compile(r"/leg/src(?=/|$)")

# The worktree's own build, relative to /src ("" for none). Its generated
# headers are the branch's; the volume's are its leg commit's.
BUILD = os.environ.get("GCC_CHECK_BUILD", "")


def generated_first(a):
    """[a], or, for a volume generated-header directory the worktree's build
    also has, that build's directory and then a."""
    if BUILD and a.startswith("-I/leg/build/") and a.endswith("/generated/spade_slang"):
        mine = "/src/" + BUILD + a[len("-I/leg/build"):]
        if os.path.isdir(mine):
            return ["-I" + mine, a]
    return [a]


volume, files = sys.argv[1], sys.argv[2:]
db = json.loads(subprocess.check_output(["ninja", "-C", "/leg/build", "-t", "compdb"], stderr=subprocess.DEVNULL))
by_file = {entry["file"]: entry for entry in db}


def compile_with(entry, source=None, replace=None, add=()):
    """Compiles with `entry`'s command line: its own file, or `source` in place
    of it; `replace` swaps whole flags; `add` appends flags. True if clean."""
    args = shlex.split(entry["command"]) if "command" in entry else list(entry["arguments"])
    if source is not None:
        own = entry["file"]
        if args.count(own) != 1:
            print(f"gcc FAIL: the template's command names its source {args.count(own)} times, not once")
            return False
        args = [("/leg/src/" + source) if a == own else a for a in args]
    cmd, i = [], 0
    while i < len(args):
        a = args[i]
        if a in DROP_ONE:
            i += 1
        elif a in DROP_TWO:
            i += 2
        else:
            a = (replace or {}).get(a, a)
            cmd += generated_first(LEG_SRC.sub("/tmp/s", a))
            i += 1
    cmd += list(add) + ["-o", "/tmp/gcc-check.o", "-Wall", "-Wextra", "-Wpedantic", "-Werror"]
    return subprocess.run(cmd, cwd=entry["directory"]).returncode == 0


def verdict(ok, f, how):
    print(f"gcc {'ok:  ' if ok else 'FAIL:'} {f} ({how})")
    return 0 if ok else 1


rc = 0
for f in files:
    entry = by_file.get("/leg/src/" + f)
    if entry is not None:
        rc |= verdict(compile_with(entry), f, "compiled with the leg's command line")
        if f in BACKEND_ON:
            replace, add = BACKEND_ON[f]
            rc |= verdict(compile_with(entry, replace=replace, add=add), f,
                          "again with the window backend on: the leg's line, SPADE_SANDBOX_HAS_GL=1, imgui's and GLFW's includes")
        if f in UNCHECKED:
            print(f"gcc note: {f}: {UNCHECKED[f]}")
        continue
    template = next((t for t in TEMPLATES if f.startswith(t[0])), None)
    if template is not None:
        prefix, model, why = template
        model_entry = by_file.get("/leg/src/" + model)
        if model_entry is None:
            print(f"gcc FAIL: {f}: {why}, and volume {volume}'s build lacks its template {model}; run a leg, then check again")
            rc = 1
            continue
        rc |= verdict(compile_with(model_entry, source=f), f, f"compiled with {model}'s command line: {why}")
        continue
    if is_new_test(f):
        model, model_entry = spade_tests_sibling(by_file)
        if model_entry is None:
            print(f"gcc FAIL: {f}: a new spade_tests file, and volume {volume}'s build compiles no spade_tests file "
                  "to borrow from; run a leg, then check again")
            rc = 1
            continue
        rc |= verdict(compile_with(model_entry, source=f), f,
                      f"compiled with {model}'s command line: a new spade_tests file the leg has not built")
        continue
    print(f"gcc FAIL: {f}: volume {volume}'s build does not compile it (a new file?); "
          "run a leg (scripts\\docker-leg.ps1), then check again")
    rc = 1
sys.exit(rc)

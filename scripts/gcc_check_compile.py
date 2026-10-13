"""scripts/gcc-check.sh --compile, inside the leg image: compile each named .cpp
in full with the leg volume's own command line (Release, -O3), so the
optimiser's warnings -- maybe-uninitialized, array-bounds and the like, which
-fsyntax-only never runs -- surface before a leg does.

    python3 gcc_check_compile.py <volume> <file.cpp>...

Run by gcc-check.sh with the volume at /leg (read-only) and the worktree's
sources copied to /tmp/s. For each file it takes the command ninja records for
that translation unit (`ninja -t compdb`), points its sources at /tmp/s, drops
the depfile flags (the volume is read-only), writes the object to /tmp, and
adds -Wall -Wextra -Wpedantic -Werror. A file the volume's build never
compiled fails with the reason: run a leg first.

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

DROP_ONE = {"-MD", "-MMD"}
DROP_TWO = {"-MF", "-MT", "-MQ", "-o"}

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
            cmd.append(a.replace("/leg/src/", "/tmp/s/"))
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
    print(f"gcc FAIL: {f}: volume {volume}'s build does not compile it (a new file?); "
          "run a leg (scripts\\docker-leg.ps1), then check again")
    rc = 1
sys.exit(rc)

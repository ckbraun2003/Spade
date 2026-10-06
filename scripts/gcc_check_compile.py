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
"""
import json
import shlex
import subprocess
import sys

volume, files = sys.argv[1], sys.argv[2:]
db = json.loads(subprocess.check_output(["ninja", "-C", "/leg/build", "-t", "compdb"], stderr=subprocess.DEVNULL))
by_file = {entry["file"]: entry for entry in db}

DROP_ONE = {"-MD", "-MMD"}
DROP_TWO = {"-MF", "-MT", "-MQ", "-o"}

rc = 0
for f in files:
    entry = by_file.get("/leg/src/" + f)
    if entry is None:
        print(f"gcc FAIL: {f}: volume {volume}'s build does not compile it (a new file?); "
              "run a leg (scripts\\docker-leg.ps1), then check again")
        rc = 1
        continue
    args = shlex.split(entry["command"]) if "command" in entry else list(entry["arguments"])
    cmd, i = [], 0
    while i < len(args):
        a = args[i]
        if a in DROP_ONE:
            i += 1
        elif a in DROP_TWO:
            i += 2
        else:
            cmd.append(a.replace("/leg/src/", "/tmp/s/"))
            i += 1
    cmd += ["-o", "/tmp/gcc-check.o", "-Wall", "-Wextra", "-Wpedantic", "-Werror"]
    run = subprocess.run(cmd, cwd=entry["directory"])
    if run.returncode == 0:
        print(f"gcc ok:   {f} (compiled with the leg's command line)")
    else:
        print(f"gcc FAIL: {f} (compiled with the leg's command line)")
        rc = 1
sys.exit(rc)

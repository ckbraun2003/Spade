"""The performance harness (tests/bench/PERF.md is the protocol; read it first).

  python tests/bench/measure.py --sha <gated sha> --tree <the measured checkout> --build-dir build-ninja/release [--metrics core.*,rendering.*]
      the harness may run from any checkout (a later one than the measured SHA); the measured tree is --tree
  python tests/bench/measure.py --dry-run ...        run and print, write nothing, accept any tree and any sha
  python tests/bench/measure.py --floors A.json B.json   the noise floor from two runs of one SHA
  python tests/bench/measure.py --trend              the ledger's last two rows per metric, judged against the floors
  python tests/bench/measure.py --table              the metrics table as Markdown (for PERF.md)

Run a real measurement as a Vantage job, as the protocol says (`vantage.py job run --class test|gpu`); the harness
reads the queue itself, excludes its own slot (its command line holds "measure.py"), and marks a run noisy and repeats
it when another heavy, gpu or quick job shows up in a reading.

Standard library only; Windows only (it reads the CPU and memory through kernel32).
"""
import argparse
import csv
import ctypes
import datetime
import fnmatch
import json
import os
import re
import shutil
import statistics
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
VANTAGE = r"C:\Users\Cameron\Desktop\Vantage\vantage.py"  # the jobs service: `job status` lists the slots (the old jobrun.py is a shim to it)
LEDGER_HEADER = ["date", "sha", "metric", "unit", "median", "min", "max", "n", "notes"]
ENV_HEADER = ["run", "sha", "harness_sha", "tree", "build_type", "compiler", "gpu", "gpu_driver", "power_plan", "free_phys_gb", "free_commit_gb",
              "cpu", "logical_cpus", "idle_cpu_pct", "queue", "noisy", "metrics"]
PERF_RE = re.compile(r"PERF\s+(\S+)\s+(\S+)\s+(\S+)\s*$")
NOTE_RE = re.compile(r"PERF-NOTE\s+(\S+)\s+(.*)$")
NS_TO = {"ns": 1.0, "us": 1e-3, "ms": 1e-6, "s": 1e-9}
MAX_NOISY_RETRIES = 3
IGNORE_QUEUE = False  # --dry-run --ignore-queue only: tests the mechanics on a busy box


def load_registry(path):
    with open(path, encoding="utf8") as f:
        return json.load(f)["metrics"]


def selected(registry, patterns, include_diagnostics=False, cls=None):
    """The metrics to run: by name, owner or glob (default: every ledger metric), and by job class (test or gpu)."""
    out = []
    for m in registry:
        if cls and m.get("class", "test") != cls:
            continue
        if not patterns:
            if include_diagnostics or not m.get("diagnostic"):
                out.append(m)
        elif any(fnmatch.fnmatch(m["name"], p) or fnmatch.fnmatch(m.get("owner", ""), p) for p in patterns):
            out.append(m)
    return out


# ---- the box -----------------------------------------------------------------------------------------------------

class _FileTime(ctypes.Structure):
    _fields_ = [("lo", ctypes.c_uint32), ("hi", ctypes.c_uint32)]


def _ticks(ft):
    return (ft.hi << 32) | ft.lo


def cpu_busy_pct(seconds):
    """Total CPU use over `seconds`, from the system's idle and total times."""
    idle, kern, user = _FileTime(), _FileTime(), _FileTime()
    k = ctypes.windll.kernel32
    k.GetSystemTimes(ctypes.byref(idle), ctypes.byref(kern), ctypes.byref(user))
    i0, t0 = _ticks(idle), _ticks(kern) + _ticks(user)
    time.sleep(seconds)
    k.GetSystemTimes(ctypes.byref(idle), ctypes.byref(kern), ctypes.byref(user))
    i1, t1 = _ticks(idle), _ticks(kern) + _ticks(user)
    total = t1 - t0
    return 100.0 * (1.0 - (i1 - i0) / total) if total > 0 else 0.0


class _MemStatus(ctypes.Structure):
    _fields_ = [("length", ctypes.c_uint32), ("load", ctypes.c_uint32), ("total_phys", ctypes.c_uint64),
                ("avail_phys", ctypes.c_uint64), ("total_page", ctypes.c_uint64), ("avail_page", ctypes.c_uint64),
                ("total_virt", ctypes.c_uint64), ("avail_virt", ctypes.c_uint64), ("avail_ext", ctypes.c_uint64)]


def free_memory_gb():
    s = _MemStatus()
    s.length = ctypes.sizeof(s)
    ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(s))
    return s.avail_phys / 2**30, s.avail_page / 2**30


def run_text(argv, timeout=60):
    try:
        return subprocess.run(argv, capture_output=True, text=True, timeout=timeout).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def environment(build_dir):
    gpu, driver = "", ""
    smi = run_text(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"])
    if smi:
        gpu, _, driver = [p.strip() for p in smi.splitlines()[0].partition(",")]
    else:
        video = run_text(["powershell", "-NoProfile", "-Command",
                          "(Get-CimInstance Win32_VideoController | Select-Object -First 1 | ForEach-Object { $_.Name + '|' + $_.DriverVersion })"])
        gpu, _, driver = video.partition("|")
    plan = run_text(["powercfg", "/getactivescheme"])
    m = re.search(r"\((.*)\)", plan)
    cpu = run_text(["powershell", "-NoProfile", "-Command", "(Get-CimInstance Win32_Processor | Select-Object -First 1).Name"])
    build_type, compiler = "", ""
    cache = os.path.join(build_dir, "CMakeCache.txt")
    if os.path.exists(cache):
        text = open(cache, encoding="utf8", errors="replace").read()
        bt = re.search(r"^CMAKE_BUILD_TYPE:\w+=(.*)$", text, re.M)
        cc = re.search(r"^CMAKE_CXX_COMPILER:\w+=(.*)$", text, re.M)
        build_type, compiler = (bt.group(1) if bt else ""), (cc.group(1) if cc else "")
    phys, commit = free_memory_gb()
    return {"build_type": build_type, "compiler": compiler, "gpu": gpu, "gpu_driver": driver,
            "power_plan": m.group(1) if m else plan, "free_phys_gb": round(phys, 1), "free_commit_gb": round(commit, 1),
            "cpu": cpu, "logical_cpus": os.cpu_count()}


def queue_others():
    """The other jobs running now (heavy, gpu or quick slots), from `vantage.py job status`, never this harness."""
    if IGNORE_QUEUE:
        return []
    if not os.path.exists(VANTAGE):
        return None
    text = run_text([sys.executable, "-I", VANTAGE, "job", "status"])
    others = []
    for line in text.splitlines():
        if re.match(r"\s+(heavy|gpu|quick)-\d+\s+pid=", line) and "measure.py" not in line:
            others.append(line.strip().split(" cmd=")[0])
    return others


def wait_idle(limit_s, pct):
    start = time.time()
    reading = 100.0
    while True:
        reading = cpu_busy_pct(10.0)
        if reading < pct:
            return reading
        if time.time() - start >= limit_s:
            return -max(reading, 1e-9)  # negative: never got under the limit; the size is the last reading
        time.sleep(2.0)


# ---- one run of a metric ----------------------------------------------------------------------------------------

def peak_working_set_mb(proc):
    class Counters(ctypes.Structure):
        _fields_ = [("cb", ctypes.c_uint32), ("page_faults", ctypes.c_uint32), ("peak_ws", ctypes.c_size_t),
                    ("ws", ctypes.c_size_t), ("a", ctypes.c_size_t), ("b", ctypes.c_size_t), ("c", ctypes.c_size_t),
                    ("d", ctypes.c_size_t), ("pagefile", ctypes.c_size_t), ("peak_pagefile", ctypes.c_size_t)]
    c = Counters()
    c.cb = ctypes.sizeof(c)
    ok = ctypes.windll.psapi.GetProcessMemoryInfo(ctypes.c_void_p(int(proc._handle)), ctypes.byref(c), c.cb)
    return c.peak_ws / 2**20 if ok else float("nan")


def expand(argv, ctx):
    return [a.format(**ctx) for a in argv]


def child_env(m, ctx):
    """This process's environment plus the metric's "env" (a null value removes the variable)."""
    env = dict(os.environ)
    for k, v in (m.get("env") or {}).items():
        if v is None:
            env.pop(k, None)
        else:
            env[k] = v.format(**ctx)
    return env


def run_process(argv, cwd, env=None):
    """Run once; returns (stdout, wall_s, peak_working_set_mb, returncode)."""
    t0 = time.perf_counter()
    proc = subprocess.Popen(argv, cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf8", errors="replace")
    out, _ = proc.communicate()
    wall = time.perf_counter() - t0
    return out, wall, peak_working_set_mb(proc), proc.returncode


def process_cpu_s(proc):
    """Kernel plus user CPU seconds the process used (the handle is still valid after the process has exited)."""
    c, e, k, u = (ctypes.c_uint64() for _ in range(4))  # a FILETIME is two little-endian 32-bit words: one uint64
    ok = ctypes.windll.kernel32.GetProcessTimes(ctypes.c_void_p(int(proc._handle)), ctypes.byref(c), ctypes.byref(e), ctypes.byref(k), ctypes.byref(u))
    return (k.value + u.value) / 1e7 if ok else float("nan")


def close_windows(pid):
    """Post WM_CLOSE to every visible top-level window of the process: the same as clicking the window's close button."""
    user32 = ctypes.windll.user32
    user32.PostMessageW.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p]
    proto = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)

    def each(hwnd, _):
        owner = ctypes.c_uint32()
        user32.GetWindowThreadProcessId(ctypes.c_void_p(hwnd), ctypes.byref(owner))
        if owner.value == pid and user32.IsWindowVisible(ctypes.c_void_p(hwnd)):
            user32.PostMessageW(hwnd, 0x0010, 0, 0)
        return True
    user32.EnumWindows(proto(each), 0)


def run_window(argv, cwd, env, hold_s, exit_timeout_s=30):
    """Launch a program that opens a window, hands off for hold_s seconds, closes it, and returns the same tuple as run_process.
    The stdout gets one more line, `HARNESS cpu_s <s>`: the CPU seconds the process used from launch to exit."""
    t0 = time.perf_counter()
    proc = subprocess.Popen(argv, cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf8", errors="replace")
    chunks = []
    reader = threading.Thread(target=lambda: chunks.append(proc.stdout.read()))
    reader.start()
    while time.perf_counter() - t0 < hold_s and proc.poll() is None:
        time.sleep(0.2)
    closing = time.perf_counter()
    while proc.poll() is None:
        if time.perf_counter() - closing > exit_timeout_s:
            proc.kill()
            proc.wait()
            reader.join()
            raise RuntimeError("the window did not close within %d s of WM_CLOSE:\n%s" % (exit_timeout_s, "".join(chunks)[-1500:]))
        close_windows(proc.pid)
        time.sleep(1.0)
    reader.join()
    wall = time.perf_counter() - t0
    out = "".join(chunks) + "\nHARNESS cpu_s %.3f\n" % process_cpu_s(proc)
    if wall < 0.9 * hold_s and proc.returncode == 0:
        raise RuntimeError("the program exited after %.1f s, before the %d s hold:\n%s" % (wall, hold_s, out[-1500:]))
    return out, wall, peak_working_set_mb(proc), proc.returncode


def gbench_json(out):
    """The google-benchmark JSON object in a program's output (text before and after it is ignored)."""
    start = out.index("{")
    return json.JSONDecoder().raw_decode(out[start:].replace("Infinity", "1e999"))[0]


def save_raw_gbench(ctx, name, out):
    """Keeps the whole benchmark JSON of a measured run (both cpu_time and real_time), so a change of basis can be re-read without a rerun."""
    if not ctx.get("raw_dir"):
        return
    os.makedirs(ctx["raw_dir"], exist_ok=True)
    with open(os.path.join(ctx["raw_dir"], "%s-%s.gbench.json" % (ctx["run_id"], name)), "w", encoding="utf8") as f:
        json.dump(gbench_json(out), f, indent=1)


def gbench_values(out, basis, unit):
    """{benchmark name: [one value per repetition]} from google-benchmark JSON, in `unit`."""
    data = gbench_json(out)
    values = {}
    for b in data["benchmarks"]:
        if b.get("run_type", "iteration") != "iteration":
            continue
        ns = b[basis] * {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}[b.get("time_unit", "ns")]
        values.setdefault(b["name"], []).append(ns * NS_TO[unit])
    return values


def add_perf(result, out, m):
    """PERF lines in a program's output (SPADE_EDITOR_PERF=1 and the like) join the rows, one value per run per name as in the perf-lines kind.
    A registry "per_occurrence" list gives the suffixes for a name printed that many times (the live smoke prints a Run air set per chapter)."""
    got = {}
    for line in out.splitlines():
        p = PERF_RE.match(line)
        if p:
            got.setdefault(p.group(1), (p.group(3), []))[1].append(float(p.group(2)))
    occ = m.get("per_occurrence") or []
    for name, (unit, vals) in got.items():
        pairs = list(zip(occ, vals)) if occ and len(vals) == len(occ) else [("", statistics.median(vals))]
        for suffix, v in pairs:
            key = name + suffix
            r = result.setdefault(key, {"unit": unit, "values": [], "basis": m.get("basis", "own"), "perf": True})
            r["diag"] = bool(m.get("diagnostic")) or ".diag_" in key
            r["values"].append(v)


def measure_metric(m, ctx, repeats, say):
    """Returns {metric name: {"unit", "values", "basis", "noisy"}}. One warm-up run is thrown away."""
    kind = m["kind"]
    repeats = m.get("repeats", repeats)  # a long run (the live smoke) says how many it takes
    argv = expand(m["command"], ctx)
    env = child_env(m, ctx)
    cwd = ctx["repo"]
    result = {}
    noisy_total = 0

    def one_run():
        nonlocal noisy_total
        for attempt in range(MAX_NOISY_RETRIES + 1):
            before = queue_others()
            if kind == "live-smoke":
                if not shutil.which("ffmpeg", path=env.get("PATH")):
                    raise RuntimeError("%s: ffmpeg is not on PATH (the smoke records the tour through it)" % m["name"])
                shutil.rmtree(ctx["smoke_out"], ignore_errors=True)  # no stale mp4 or journal can pass for this run
            if kind == "editor-idle":
                out, wall, peak, code = run_window(argv, cwd, env, float(m.get("hold_s", 30)))
            else:
                out, wall, peak, code = run_process(argv, cwd, env)
            if kind == "live-smoke":
                journal = os.path.join(ctx["smoke_out"], "journal.txt")
                out += "\nJOURNAL\n" + (open(journal, encoding="utf8", errors="replace").read() if os.path.exists(journal) else "")
            after = queue_others()
            seen = sorted(set((before or []) + (after or [])))
            if code != 0:
                raise RuntimeError("%s: exit %d\n%s" % (m["name"], code, out[-2000:]))
            if seen and attempt < MAX_NOISY_RETRIES:
                noisy_total += 1
                say("  noisy (%s), repeating" % seen[0])
                continue
            return out, wall, peak, seen
        raise AssertionError("unreachable")

    say("%s: warm-up" % m["name"])
    one_run()
    if kind == "gbench":
        # one process makes all repetitions, so a run is the whole invocation
        out, wall, peak, seen = one_run()
        save_raw_gbench(ctx, m["name"], out)
        div = float(m.get("divisor", 1))  # an iteration that makes N steps is reported per step with "divisor": N
        for name, vals in gbench_values(out, m["basis"], m["unit"]).items():
            _, _, arg = name.partition("/")  # BM_StepPlainBodies/1000 -> <registry name>/1000
            label = m["name"] + ("/" + arg if arg else "")
            if label in result:
                raise RuntimeError("%s: two benchmarks give the ledger name %s (anchor the filter, for example ^BM_Name/)" % (m["name"], label))
            result[label] = {"unit": m["unit"], "values": [v / div for v in vals], "basis": m["basis"], "diag": bool(m.get("diagnostic"))}
    else:
        runs = []
        for i in range(repeats):
            say("%s: repeat %d/%d" % (m["name"], i + 1, repeats))
            runs.append(one_run())
        if kind == "perf-lines":
            # One value per run per name: a name printed several times in one run (one line per timed step) is that run's median.
            notes = {}
            for out, wall, peak, seen in runs:
                got = {}
                for line in out.splitlines():
                    p = PERF_RE.match(line)
                    if p:
                        got.setdefault(p.group(1), (p.group(3), []))[1].append(float(p.group(2)))
                    n = NOTE_RE.match(line)
                    if n:
                        notes[n.group(1)] = re.sub(r"[;,\s]+", "_", n.group(2).strip())
                for name, (unit, vals) in got.items():
                    r = result.setdefault(name, {"unit": unit, "values": [], "basis": m.get("basis", "own")})
                    r["values"].append(statistics.median(vals))
            if not result:
                raise RuntimeError("%s printed no PERF lines (is its env var set, and is the test built?)\n%s" % (m["name"], runs[-1][0][-1500:]))
            note = ";".join("%s=%s" % kv for kv in sorted(notes.items()))
            for name, r in result.items():
                r["diag"] = bool(m.get("diagnostic")) or ".diag_" in name
                if note:
                    r["note"] = note
                if len(r["values"]) != repeats:
                    say("  warning: %s has %d values for %d repeats" % (name, len(r["values"]), repeats))
        elif kind == "live-smoke":
            # the wall time of the whole tour (process launch to exit, ffmpeg's finish included), against the video it makes
            name = m["name"]
            notes = ""
            for out, wall, peak, seen in runs:
                note = re.search(r"frames (\d+), recorded (\d+) \((\d+) s of video\)", out)
                if not note:
                    raise RuntimeError("%s: the journal has no 'frames ..., recorded ... (N s of video)' line:\n%s" % (name, out[-1500:]))
                air = [float(x) for x in re.findall(r"frames in ([\d.]+) s of wall time", out)]
                rows = {name + "_wall": ("s", wall, False),
                        name + ".diag_video": ("s", float(note.group(3)), True),
                        name + ".diag_frames": ("frames", float(note.group(1)), True),
                        name + ".diag_peak_mb": ("MB", peak, True)}
                if air:
                    rows[name + ".diag_air_wall"] = ("s", sum(air), True)  # the Run air chapters' frames, wall seconds
                for key, (unit, value, diag) in rows.items():
                    r = result.setdefault(key, {"unit": unit, "values": [], "basis": "wall", "diag": diag})
                    r["values"].append(value)
                add_perf(result, out, m)
        elif kind == "editor-idle":
            # the sandbox's exit summary (sandbox/main.cpp print_exit_summary) after a hands-off hold, plus the process's CPU seconds
            name = m["name"]
            hold = float(m.get("hold_s", 30))
            for out, wall, peak, seen in runs:
                mean = re.search(r"mean frame ([\d.]+) ms \(([\d.]+) fps\)", out)
                split = re.search(r"physics ([\d.]+)\s+render ([\d.]+)\s+convert ([\d.]+)\s+upload ([\d.]+)\s+ui ([\d.]+)\s+swap ([\d.]+)\s+ms", out)
                frames = re.search(r"closed after (\d+) frames", out)
                ws = re.search(r"working set ([\d.]+) MB", out)
                cpu = re.search(r"HARNESS cpu_s ([\d.]+)", out)
                if not (mean and split and frames and cpu):
                    raise RuntimeError("%s: the exit summary is missing:\n%s" % (name, out[-1500:]))
                rows = {name + "_frame": ("ms", float(mean.group(1)), False),
                        name + "_cpu": ("s", float(cpu.group(1)), False),
                        name + ".diag_frames": ("frames", float(frames.group(1)), True)}
                for k, part in enumerate(("physics", "render", "convert", "upload", "ui", "swap")):
                    rows["%s.diag_%s" % (name, part)] = ("ms", float(split.group(k + 1)), True)
                if ws:
                    rows[name + ".diag_working_set"] = ("MB", float(ws.group(1)), True)
                for key, (unit, value, diag) in rows.items():
                    r = result.setdefault(key, {"unit": unit, "values": [], "basis": "own", "diag": diag})
                    r["values"].append(value)
                add_perf(result, out, m)
            for r in result.values():
                r["note"] = "hold_s=%g;mean_is_lifetime" % hold
        elif kind == "wall":
            result[m["name"]] = {"unit": "s", "values": [r[1] for r in runs], "basis": "wall"}
            result[m["name"] + ".peak_mb"] = {"unit": "MB", "values": [r[2] for r in runs], "basis": "peak_working_set"}
        else:
            raise ValueError("unknown kind %r" % kind)
    for key, r in result.items():
        r["noisy"] = noisy_total
        if r.get("perf") and len(r["values"]) != repeats:
            say("  warning: %s has %d values for %d repeats" % (key, len(r["values"]), repeats))
    return result


def summary(values):
    return {"median": statistics.median(values), "min": min(values), "max": max(values), "n": len(values)}


# ---- the ledger ---------------------------------------------------------------------------------------------------

def append_rows(path, header, rows):
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", encoding="utf8", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        if new:
            w.writerow(header)
        w.writerows(rows)


def git(tree, *args):
    return run_text(["git", "-C", tree] + list(args))


def cmd_measure(a):
    registry = load_registry(a.registry)
    chosen = selected(registry, a.metrics.split(",") if a.metrics else None, a.include_diagnostics, a.job_class)
    if not chosen:
        sys.exit("no metric matches %r" % a.metrics)
    say = lambda s: print(s, flush=True)
    tree = os.path.abspath(a.tree) if a.tree else REPO
    harness_sha = git(REPO, "rev-parse", "HEAD")[:10]
    sha = (a.sha or git(tree, "rev-parse", "HEAD"))[:10]
    if not a.dry_run:
        if not a.sha:
            sys.exit("--sha <the gated sha> is required: a number is never taken on an ungated tree")
        head = git(tree, "rev-parse", "HEAD")
        if not head.startswith(a.sha):
            sys.exit("the tree %s is at %s, not the gated sha %s" % (tree, head[:10] or "(not a git tree)", a.sha))
        if git(tree, "status", "--porcelain", "--untracked-files=no"):
            sys.exit("the tree %s has uncommitted changes: refusing to measure" % tree)
    build_dir = os.path.abspath(os.path.join(tree, a.build_dir))
    if not a.dry_run and os.path.commonpath([build_dir, tree]) != tree:
        sys.exit("the build folder %s is not inside the tree %s: the binaries must come from the measured tree" % (build_dir, tree))
    others = queue_others()
    if others is None:
        say("warning: vantage.py not found at %s; the queue is not checked" % VANTAGE)
    elif others:
        say("the queue is busy:\n  " + "\n  ".join(others))
        if not a.dry_run:
            sys.exit("another heavy, gpu or quick job is running: not a quiet window")
    say("idle check (under %.0f%% CPU over 10 s, up to %d s)" % (a.idle_pct, a.idle_limit_s))
    idle = wait_idle(a.idle_limit_s, a.idle_pct)
    if idle is not None and idle < 0:
        sys.exit("the box never got under %.0f%% CPU in %d s; last reading %.1f%%. Not measured." % (a.idle_pct, a.idle_limit_s, -idle))
    today = datetime.date.today().isoformat()
    run_id = "%s_%s" % (today, sha[:7])
    exe = lambda name: os.path.join(build_dir, "bin", name + ".exe")
    ctx = {"repo": tree, "build": build_dir, "bench": exe("spade_bench"), "tests": exe("spade_tests"), "flow_movie": exe("flow_movie"), "sandbox": exe("spade_sandbox"),
           "smoke_out": os.path.join(REPO, "build-bench", "perf-smoke"),
           "ticket": "perf-" + run_id, "run_id": run_id,
           "raw_dir": os.path.abspath(a.raw_dir or os.path.dirname(os.path.abspath(a.out_json or a.ledger)))}
    env = environment(build_dir)
    queue_readings = []
    results, noisy = {}, 0
    for m in chosen:
        res = measure_metric(m, ctx, a.repeats, say)
        results.update(res)
        noisy += max((r["noisy"] for r in res.values()), default=0)
    after = queue_others()
    queue_readings.append("clean" if not others and not after else "busy:" + ";".join((others or []) + (after or [])))
    rows = []
    for name in sorted(results):
        r = results[name]
        s = summary(r["values"])
        if r.get("diag"):
            continue
        notes = "run=%s;basis=%s;noisy=%d%s" % (run_id, r["basis"], r["noisy"], (";" + r["note"]) if r.get("note") else "")
        rows.append([today, sha, name, r["unit"], "%.6g" % s["median"], "%.6g" % s["min"], "%.6g" % s["max"], s["n"], notes])
    for row in rows:
        say("  %-52s %12s %s  [%s .. %s]  n=%s" % (row[2], row[4], row[3], row[5], row[6], row[7]))
    if a.out_json:
        os.makedirs(os.path.dirname(os.path.abspath(a.out_json)), exist_ok=True)
        with open(a.out_json, "w", encoding="utf8") as f:
            json.dump({"run": run_id, "sha": sha, "env": env, "results": results}, f, indent=1)
    if a.dry_run:
        say("dry run: nothing written (%d rows)" % len(rows))
        return
    append_rows(a.ledger, LEDGER_HEADER, rows)
    append_rows(a.env, ENV_HEADER, [[run_id, sha, harness_sha, tree, env["build_type"], env["compiler"], env["gpu"], env["gpu_driver"],
                                     env["power_plan"], env["free_phys_gb"], env["free_commit_gb"], env["cpu"],
                                     env["logical_cpus"], "%.1f" % (idle if idle is not None else -1), ",".join(queue_readings), noisy,
                                     len(rows)]])
    say("wrote %d ledger rows and the environment row for %s" % (len(rows), run_id))


# ---- the noise floor, the trend, the table ---------------------------------------------------------------------

def cmd_floors(a):
    A, B = (json.load(open(p, encoding="utf8")) for p in a.floors)
    if A["sha"] != B["sha"]:
        sys.exit("the two runs are of different SHAs (%s, %s)" % (A["sha"], B["sha"]))
    floors = {}
    for name, ra in A["results"].items():
        rb = B["results"].get(name)
        if rb is None:
            continue
        sa, sb = summary(ra["values"]), summary(rb["values"])
        between = abs(sa["median"] - sb["median"]) / sa["median"] if sa["median"] else 0.0
        within = max((s["max"] - s["min"]) / s["median"] for s in (sa, sb)) if sa["median"] and sb["median"] else 0.0
        floors[name] = round(100.0 * max(between, within), 2)
        print("%-52s between %.2f%%  within %.2f%%  floor %.2f%%" % (name, 100 * between, 100 * within, floors[name]))
    if a.out_floors:
        if os.path.exists(a.out_floors):  # the test-class and gpu-class runs are separate invocations: merge, never drop
            floors = {**json.load(open(a.out_floors, encoding="utf8")), **floors}
        with open(a.out_floors, "w", encoding="utf8") as f:
            json.dump(floors, f, indent=1, sort_keys=True)
        print("floors written to", a.out_floors)


def cmd_trend(a):
    floors = json.load(open(a.floors_file, encoding="utf8")) if os.path.exists(a.floors_file) else {}
    rows = list(csv.DictReader(open(a.ledger, encoding="utf8")))
    by = {}
    for r in rows:
        by.setdefault(r["metric"], []).append(r)
    for name, rs in sorted(by.items()):
        if len(rs) < 2:
            print("%-52s one row" % name)
            continue
        ref, new = rs[-2], rs[-1]
        rm, nm = float(ref["median"]), float(new["median"])
        change = 100.0 * (nm - rm) / rm if rm else 0.0
        overlap = not (float(new["min"]) > float(ref["max"]) or float(new["max"]) < float(ref["min"]))
        floor = floors.get(name)
        if floor is None:
            verdict = "no floor measured"
        elif abs(change) > floor and not overlap:
            verdict = "REAL %s" % ("slower" if nm > rm else "faster") if ref["unit"] not in ("fps",) else "REAL %s" % ("faster" if nm > rm else "slower")
        else:
            verdict = "noise"
        print("%-52s %+7.2f%%  (%s -> %s %s)  %s" % (name, change, ref["sha"][:7], new["sha"][:7], new["unit"], verdict))


def cmd_table(a):
    floors = json.load(open(a.floors_file, encoding="utf8")) if os.path.exists(a.floors_file) else {}
    print("| Metric | Who waits, for what | Workload | Command | Unit | Owner | Kind | Class | Floor |")
    print("|---|---|---|---|---|---|---|---|---|")
    registry = load_registry(a.registry)
    every_prefix = [p for r in registry for p in [r["name"]] + list(r.get("row_prefixes") or [])]
    for m in registry:
        # A metric's rows are the floors whose name starts with the metric's (or a registry "row_prefixes" entry), the next character
        # one of / _ . ; the phase diagnostics (.diag_) are not rows. The table shows the largest row floor, "up to" when there are several.
        prefixes = [m["name"]] + list(m.get("row_prefixes") or [])
        own = [v for k, v in floors.items()
               if ".diag_" not in k and any(k == p or (k.startswith(p) and k[len(p)] in "/_.") for p in prefixes)
               and not any(len(q) > len(max(prefixes, key=len)) and k.startswith(q) for q in every_prefix)]
        if own:
            f = ("up to %.1f%%" if len(own) > 1 else "%.1f%%") % max(own)
        elif m.get("class", "test") == "test" and not m.get("diagnostic"):
            f = "pending the second window"
        else:
            f = "not measured"
        cmd = " ".join(["%s=%s" % (k, v) for k, v in (m.get("env") or {}).items() if v is not None] + m["command"])
        print("| `%s` | %s | %s | `%s` | %s | %s | %s | %s | %s |" % (
            m["name"], m.get("who_waits", ""), m.get("workload", ""), cmd.replace("|", "\\|"), m["unit"],
            m.get("owner", ""), "diagnostic" if m.get("diagnostic") else "metric", m.get("class", "test"), f))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sha", help="the gated SHA (required unless --dry-run)")
    p.add_argument("--tree", help="the checkout and build being measured (default: this harness's own checkout). Its HEAD must equal --sha and it must have no uncommitted change; --build-dir is inside it")
    p.add_argument("--build-dir", default=os.path.join("build-ninja", "release"))
    p.add_argument("--metrics", help="comma-separated names, owners or globs (default: every ledger metric)")
    p.add_argument("--class", dest="job_class", choices=["test", "gpu"], help="only the metrics of this job class (run each class as its own job)")
    p.add_argument("--repeats", type=int, default=5)
    p.add_argument("--registry", default=os.path.join(HERE, "metrics.json"))
    p.add_argument("--ledger", default=os.path.join(HERE, "ledger.csv"))
    p.add_argument("--env", default=os.path.join(HERE, "environment.csv"))
    p.add_argument("--floors-file", default=os.path.join(HERE, "floors.json"))
    p.add_argument("--out-json", help="write the raw values here (for --floors)")
    p.add_argument("--raw-dir", help="where each benchmark run's whole JSON goes (default: next to --out-json, else next to the ledger)")
    p.add_argument("--idle-pct", type=float, default=5.0)
    p.add_argument("--idle-limit-s", type=int, default=600)
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--ignore-queue", action="store_true", help="with --dry-run: do not read the queue (mechanics test on a busy box)")
    p.add_argument("--include-diagnostics", action="store_true", help="also run the diagnostics (never in the ledger)")
    p.add_argument("--floors", nargs=2, metavar=("A.json", "B.json"))
    p.add_argument("--out-floors")
    p.add_argument("--trend", action="store_true")
    p.add_argument("--table", action="store_true")
    a = p.parse_args()
    if a.ignore_queue:
        if not a.dry_run:
            sys.exit("--ignore-queue is for --dry-run only")
        global IGNORE_QUEUE
        IGNORE_QUEUE = True
    if a.floors:
        cmd_floors(a)
    elif a.trend:
        cmd_trend(a)
    elif a.table:
        cmd_table(a)
    else:
        cmd_measure(a)


if __name__ == "__main__":
    main()

"""The performance harness (tests/bench/PERF.md is the protocol; read it first).

  python tests/bench/measure.py --sha <gated sha> --build-dir build-ninja/release [--metrics core.*,rendering.*]
  python tests/bench/measure.py --dry-run ...        run and print, write nothing, accept any tree and any sha
  python tests/bench/measure.py --floors A.json B.json   the noise floor from two runs of one SHA
  python tests/bench/measure.py --trend              the ledger's last two rows per metric, judged against the floors
  python tests/bench/measure.py --table              the metrics table as Markdown (for PERF.md)

Run a real measurement under jobrun, as the protocol says (a heavy job, with --gpu for a GPU metric); the harness
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
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
JOBRUN = r"C:\Users\Cameron\Desktop\Projects\kat-wt\_handoff\tools\jobrun.py"
LEDGER_HEADER = ["date", "sha", "metric", "unit", "median", "min", "max", "n", "notes"]
ENV_HEADER = ["run", "sha", "build_type", "compiler", "gpu", "gpu_driver", "power_plan", "free_phys_gb", "free_commit_gb",
              "cpu", "logical_cpus", "idle_cpu_pct", "queue", "noisy", "metrics"]
NS_TO = {"ns": 1.0, "us": 1e-3, "ms": 1e-6, "s": 1e-9}
MAX_NOISY_RETRIES = 3
IGNORE_QUEUE = False  # --dry-run --ignore-queue only: tests the mechanics on a busy box


def load_registry(path):
    with open(path, encoding="utf8") as f:
        return json.load(f)["metrics"]


def selected(registry, patterns, include_diagnostics=False):
    if not patterns:
        return [m for m in registry if include_diagnostics or not m.get("diagnostic")]
    out = []
    for m in registry:
        if any(fnmatch.fnmatch(m["name"], p) or fnmatch.fnmatch(m.get("owner", ""), p) for p in patterns):
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
    """The other jobs running now (heavy, gpu or quick slots), from jobrun --queue-status, never this harness."""
    if IGNORE_QUEUE:
        return []
    if not os.path.exists(JOBRUN):
        return None
    text = run_text([sys.executable, "-I", JOBRUN, "--queue-status"])
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


def run_process(argv, cwd):
    """Run once; returns (stdout, wall_s, peak_working_set_mb, returncode)."""
    t0 = time.perf_counter()
    proc = subprocess.Popen(argv, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf8", errors="replace")
    out, _ = proc.communicate()
    wall = time.perf_counter() - t0
    return out, wall, peak_working_set_mb(proc), proc.returncode


def gbench_values(out, basis, unit):
    """{benchmark name: [one value per repetition]} from google-benchmark JSON, in `unit`."""
    start = out.index("{")
    data = json.JSONDecoder().raw_decode(out[start:].replace("Infinity", "1e999"))[0]
    values = {}
    for b in data["benchmarks"]:
        if b.get("run_type", "iteration") != "iteration":
            continue
        ns = b[basis] * {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}[b.get("time_unit", "ns")]
        values.setdefault(b["name"], []).append(ns * NS_TO[unit])
    return values


def measure_metric(m, ctx, repeats, say):
    """Returns {metric name: {"unit", "values", "basis", "noisy"}}. One warm-up run is thrown away."""
    kind = m["kind"]
    argv = expand(m["command"], ctx)
    cwd = ctx["repo"]
    result = {}
    noisy_total = 0

    def one_run():
        nonlocal noisy_total
        for attempt in range(MAX_NOISY_RETRIES + 1):
            before = queue_others()
            out, wall, peak, code = run_process(argv, cwd)
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
        for name, vals in gbench_values(out, m["basis"], m["unit"]).items():
            _, _, arg = name.partition("/")  # BM_StepPlainBodies/1000 -> <registry name>/1000
            label = m["name"] + ("/" + arg if arg else "")
            result[label] = {"unit": m["unit"], "values": vals, "basis": m["basis"], "diag": bool(m.get("diagnostic"))}
    else:
        runs = []
        for i in range(repeats):
            say("%s: repeat %d/%d" % (m["name"], i + 1, repeats))
            runs.append(one_run())
        if kind == "perf-lines":
            for out, wall, peak, seen in runs:
                for line in out.splitlines():
                    p = re.match(r"PERF\s+(\S+)\s+(\S+)\s+(\S+)\s*$", line)
                    if p:
                        r = result.setdefault(p.group(1), {"unit": p.group(3), "values": [], "basis": "own"})
                        r["values"].append(float(p.group(2)))
        elif kind == "wall":
            result[m["name"]] = {"unit": "s", "values": [r[1] for r in runs], "basis": "wall"}
            result[m["name"] + ".peak_mb"] = {"unit": "MB", "values": [r[2] for r in runs], "basis": "peak_working_set"}
        else:
            raise ValueError("unknown kind %r" % kind)
    for r in result.values():
        r["noisy"] = noisy_total
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


def git(*args):
    return run_text(["git", "-C", REPO] + list(args))


def cmd_measure(a):
    registry = load_registry(a.registry)
    chosen = selected(registry, a.metrics.split(",") if a.metrics else None, a.include_diagnostics)
    if not chosen:
        sys.exit("no metric matches %r" % a.metrics)
    say = lambda s: print(s, flush=True)
    sha = (a.sha or git("rev-parse", "HEAD"))[:10]
    if not a.dry_run:
        if not a.sha:
            sys.exit("--sha <the gated sha> is required: a number is never taken on an ungated tree")
        head = git("rev-parse", "HEAD")
        if not head.startswith(a.sha):
            sys.exit("HEAD is %s, not the gated sha %s" % (head[:10], a.sha))
        if git("status", "--porcelain", "--untracked-files=no"):
            sys.exit("the tree has uncommitted changes: refusing to measure")
    others = queue_others()
    if others is None:
        say("warning: jobrun not found at %s; the queue is not checked" % JOBRUN)
    elif others:
        say("the queue is busy:\n  " + "\n  ".join(others))
        if not a.dry_run:
            sys.exit("another heavy, gpu or quick job is running: not a quiet window")
    say("idle check (under %.0f%% CPU over 10 s, up to %d s)" % (a.idle_pct, a.idle_limit_s))
    idle = wait_idle(a.idle_limit_s, a.idle_pct)
    if idle is not None and idle < 0:
        sys.exit("the box never got under %.0f%% CPU in %d s; last reading %.1f%%. Not measured." % (a.idle_pct, a.idle_limit_s, -idle))
    build_dir = os.path.join(REPO, a.build_dir) if not os.path.isabs(a.build_dir) else a.build_dir
    ctx = {"repo": REPO, "build": build_dir, "bench": os.path.join(build_dir, "bin", "spade_bench.exe")}
    env = environment(build_dir)
    today = datetime.date.today().isoformat()
    run_id = "%s_%s" % (today, sha[:7])
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
        notes = "run=%s;basis=%s;noisy=%d" % (run_id, r["basis"], r["noisy"])
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
    append_rows(a.env, ENV_HEADER, [[run_id, sha, env["build_type"], env["compiler"], env["gpu"], env["gpu_driver"],
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
    print("| Metric | Who waits, for what | Workload | Command | Unit | Owner | Kind | Floor |")
    print("|---|---|---|---|---|---|---|---|")
    for m in load_registry(a.registry):
        own = [v for k, v in floors.items() if k == m["name"] or k.startswith(m["name"] + "/")]
        f = max(own) if own else None
        print("| `%s` | %s | %s | `%s` | %s | %s | %s | %s |" % (
            m["name"], m.get("who_waits", ""), m.get("workload", ""), " ".join(m["command"]).replace("|", "\\|"), m["unit"],
            m.get("owner", ""), "diagnostic" if m.get("diagnostic") else "metric", ("%.1f%%" % f) if f is not None else "not measured"))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sha", help="the gated SHA (required unless --dry-run)")
    p.add_argument("--build-dir", default=os.path.join("build-ninja", "release"))
    p.add_argument("--metrics", help="comma-separated names, owners or globs (default: every ledger metric)")
    p.add_argument("--repeats", type=int, default=5)
    p.add_argument("--registry", default=os.path.join(HERE, "metrics.json"))
    p.add_argument("--ledger", default=os.path.join(HERE, "ledger.csv"))
    p.add_argument("--env", default=os.path.join(HERE, "environment.csv"))
    p.add_argument("--floors-file", default=os.path.join(HERE, "floors.json"))
    p.add_argument("--out-json", help="write the raw values here (for --floors)")
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

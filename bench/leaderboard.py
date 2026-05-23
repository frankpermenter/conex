#!/usr/bin/env python3
"""Compare benchmark JSON files and report leaderboard/regressions.

Usage:
  python3 leaderboard.py results.json                    # Show summary
  python3 leaderboard.py baseline.json current.json      # Compare two runs
"""
import json, sys

def load(path):
    with open(path) as f:
        return json.load(f)

def summary(data):
    """Print per-algorithm solve counts and best algorithm per instance."""
    algo_stats = {}  # algo_name -> {solved, total, total_ms}
    failures = []

    for inst in data["instances"]:
        best_mu = 1e30
        best_algo = None
        for a in inst["algorithms"]:
            name = a["name"]
            if name not in algo_stats:
                algo_stats[name] = {"solved": 0, "total": 0, "total_ms": 0}
            algo_stats[name]["total"] += 1
            algo_stats[name]["total_ms"] += a["time_ms"]
            if a["converged"]:
                algo_stats[name]["solved"] += 1
            if a["mu"] < best_mu:
                best_mu = a["mu"]
                best_algo = name

        # Check if any algorithm solved it
        any_solved = any(a["converged"] for a in inst["algorithms"])
        if not any_solved:
            failures.append(inst["name"])

    print(f"Benchmark: {data['git_sha']} ({data['timestamp']})")
    print(f"Config: {json.dumps(data['config'])}")
    print(f"Instances: {len(data['instances'])}")
    print()
    print(f"{'Algorithm':20s} {'Solved':>8s} {'Total':>8s} {'Rate':>8s} {'Time(ms)':>10s}")
    print("-" * 58)
    for name in sorted(algo_stats):
        s = algo_stats[name]
        rate = f"{100*s['solved']/s['total']:.0f}%" if s['total'] > 0 else "N/A"
        print(f"{name:20s} {s['solved']:8d} {s['total']:8d} {rate:>8s} {s['total_ms']:10.0f}")

    if failures:
        print(f"\nUnsolved by any algorithm ({len(failures)}):")
        for f in failures[:20]:
            print(f"  {f}")
        if len(failures) > 20:
            print(f"  ... and {len(failures)-20} more")

def compare(baseline, current):
    """Compare two runs and report regressions/improvements."""
    # Build lookup: instance_name -> {algo_name -> converged}
    def build_map(data):
        m = {}
        for inst in data["instances"]:
            algos = {}
            for a in inst["algorithms"]:
                algos[a["name"]] = a
            m[inst["name"]] = algos
        return m

    base_map = build_map(baseline)
    curr_map = build_map(current)

    print(f"Baseline: {baseline['git_sha']} ({baseline['timestamp']})")
    print(f"Current:  {current['git_sha']} ({current['timestamp']})")
    print()

    # Per-algorithm comparison
    all_algos = set()
    for m in [base_map, curr_map]:
        for algos in m.values():
            all_algos.update(algos.keys())

    all_instances = sorted(set(base_map) | set(curr_map))

    for algo in sorted(all_algos):
        regressions = []
        improvements = []
        base_solved = 0
        curr_solved = 0

        for inst in all_instances:
            b = base_map.get(inst, {}).get(algo)
            c = curr_map.get(inst, {}).get(algo)
            b_conv = b["converged"] if b else False
            c_conv = c["converged"] if c else False
            if b: base_solved += b_conv
            if c: curr_solved += c_conv

            if b_conv and not c_conv:
                regressions.append(inst)
            elif c_conv and not b_conv:
                improvements.append(inst)

        n_base = sum(1 for i in all_instances if algo in base_map.get(i, {}))
        n_curr = sum(1 for i in all_instances if algo in curr_map.get(i, {}))
        print(f"{algo}: {base_solved}/{n_base} -> {curr_solved}/{n_curr}", end="")
        if regressions:
            print(f"  REGRESSIONS: {regressions}", end="")
        if improvements:
            print(f"  improvements: {improvements}", end="")
        print()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    elif len(sys.argv) == 2:
        summary(load(sys.argv[1]))
    else:
        compare(load(sys.argv[1]), load(sys.argv[2]))

#!/usr/bin/env python3
"""
Visualize ThetaContR evaluation results.

Usage:
  python3 plot_eval_thetacontr.py [eval_thetacontr.csv]

Produces PNG plots:
  1. scatter_fac_vs_stat.png: factorizations vs stationarity per policy
  2. performance_profile.png: fraction solved to accuracy vs fac budget
  3. Also prints ASCII tables to stdout.
"""
import sys
import csv
import math
from collections import defaultdict

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("matplotlib not found, skipping PNG plots")

def load_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            row['factorizations'] = int(row['factorizations'])
            row['iterations'] = int(row['iterations'])
            row['stationarity'] = float(row['stationarity'])
            row['complementarity'] = float(row['complementarity'])
            row['mu'] = float(row['mu'])
            row['time_ms'] = float(row['time_ms'])
            rows.append(row)
    return rows

def plot_scatter(rows):
    """Scatter: factorizations vs log10(stationarity) per config."""
    configs = sorted(set(r['config'] for r in rows))
    markers = ['o', 's', '^', 'D', 'v', '<', '>', 'p', '*', 'h']
    colors = plt.cm.tab10.colors

    for x_key, x_label, fname in [
        ('factorizations', 'Factorizations', 'scatter_fac_vs_stat.png'),
        ('time_ms', 'Wall Clock Time (ms)', 'scatter_time_vs_stat.png'),
    ]:
        fig, ax = plt.subplots(figsize=(12, 7))
        for i, c in enumerate(configs):
            crows = [r for r in rows if r['config'] == c and r['stationarity'] > 0]
            x = [r[x_key] for r in crows]
            y = [math.log10(r['stationarity']) for r in crows]
            ax.scatter(x, y, label=c, marker=markers[i % len(markers)],
                       color=colors[i % len(colors)], alpha=0.7, s=40)

        ax.set_xlabel(x_label)
        ax.set_ylabel('log10(stationarity)')
        ax.set_title(f'ThetaContR: {x_label} vs Solution Quality')
        ax.legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=-8, color='gray', linestyle='--', alpha=0.5)
        fig.tight_layout()
        fig.savefig(fname, dpi=150)
        print(f"Wrote {fname}")

def plot_performance_profile(rows):
    """Performance profile: for each fac budget, fraction of problems
    solved to stat < 1e-4."""
    configs = sorted(set(r['config'] for r in rows))
    colors = plt.cm.tab10.colors
    linestyles = ['-', '--', '-.', ':', '-', '--', '-.', ':', '-']

    thresholds = [1e-2, 1e-6, 1e-10]
    fig, axes = plt.subplots(1, len(thresholds), figsize=(5*len(thresholds), 5))
    if len(thresholds) == 1:
        axes = [axes]

    budgets = list(range(1, 51))

    for ax, thresh in zip(axes, thresholds):
        for i, c in enumerate(configs):
            crows = [r for r in rows if r['config'] == c]
            n_total = len(crows)
            fracs = []
            for b in budgets:
                solved = sum(1 for r in crows
                             if r['factorizations'] <= b
                             and r['stationarity'] >= 0
                             and r['stationarity'] < thresh)
                fracs.append(solved / max(n_total, 1))
            ax.plot(budgets, fracs, label=c,
                    color=colors[i % len(colors)],
                    linestyle=linestyles[i % len(linestyles)],
                    linewidth=1.5)
        ax.set_xlabel('Max Factorizations')
        ax.set_ylabel('Fraction Solved')
        ax.set_title(f'stat < {thresh:.0e}')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.05, 1.05)
        ax.set_xlim(0, 30)

    axes[-1].legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize=7)
    fig.suptitle('Performance Profile: ThetaContR Configurations', y=1.02)
    fig.tight_layout()
    fig.savefig('performance_profile.png', dpi=150, bbox_inches='tight')
    print("Wrote performance_profile.png")

def plot_performance_profile_time(rows):
    """Performance profile by wall clock time."""
    configs = sorted(set(r['config'] for r in rows))
    colors = plt.cm.tab10.colors
    linestyles = ['-', '--', '-.', ':', '-', '--', '-.', ':', '-']

    thresholds = [1e-2, 1e-6, 1e-10]
    fig, axes = plt.subplots(1, len(thresholds), figsize=(5*len(thresholds), 5))
    if len(thresholds) == 1:
        axes = [axes]

    time_budgets = [1, 2, 5, 10, 15, 20, 30, 50]

    for ax, thresh in zip(axes, thresholds):
        for i, c in enumerate(configs):
            crows = [r for r in rows if r['config'] == c]
            n_total = len(crows)
            fracs = []
            for t in time_budgets:
                solved = sum(1 for r in crows
                             if r['time_ms'] <= t
                             and r['stationarity'] >= 0
                             and r['stationarity'] < thresh)
                fracs.append(solved / max(n_total, 1))
            ax.plot(time_budgets, fracs, label=c,
                    color=colors[i % len(colors)],
                    linestyle=linestyles[i % len(linestyles)],
                    linewidth=1.5)
        ax.set_xlabel('Max Time (ms)')
        ax.set_ylabel('Fraction Solved')
        ax.set_title(f'stat < {thresh:.0e}')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.05, 1.05)

    axes[-1].legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize=7)
    fig.suptitle('Performance Profile by Time: ThetaContR Configurations', y=1.02)
    fig.tight_layout()
    fig.savefig('performance_profile_time.png', dpi=150, bbox_inches='tight')
    print("Wrote performance_profile_time.png")

def plot_per_problem(rows):
    """Per-problem bar chart: factorizations by config."""
    configs = sorted(set(r['config'] for r in rows))
    problems = sorted(set(r['problem'] for r in rows))
    colors = plt.cm.tab10.colors

    fig, ax = plt.subplots(figsize=(max(14, len(problems)*0.8), 6))
    n_configs = len(configs)
    width = 0.8 / n_configs
    x = range(len(problems))

    for i, c in enumerate(configs):
        facs = []
        for p in problems:
            match = [r for r in rows if r['problem'] == p and r['config'] == c]
            facs.append(match[0]['factorizations'] if match else 0)
        offset = (i - n_configs/2 + 0.5) * width
        bars = ax.bar([xi + offset for xi in x], facs, width,
                      label=c, color=colors[i % len(colors)], alpha=0.8)

    ax.set_xticks(x)
    ax.set_xticklabels(problems, rotation=45, ha='right', fontsize=7)
    ax.set_ylabel('Factorizations')
    ax.set_title('Factorizations per Problem and Config')
    ax.legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize=7)
    ax.grid(True, axis='y', alpha=0.3)
    fig.tight_layout()
    fig.savefig('fac_per_problem.png', dpi=150, bbox_inches='tight')
    print("Wrote fac_per_problem.png")

def plot_theta_convergence(rows):
    """Plot theta vs iteration for each problem, comparing configs."""
    import os
    import glob

    configs = sorted(set(r['config'] for r in rows))
    problems = sorted(set(r['problem'] for r in rows))
    colors = plt.cm.tab10.colors
    linestyles = ['-', '--', '-.', ':', '-', '--', '-.', ':', '-']

    # Find all theta trace files.
    trace_files = glob.glob("theta_trace_*.csv")
    if not trace_files:
        print("No theta_trace_*.csv files found, skipping theta plots")
        return

    # Group by problem.
    traces = defaultdict(dict)  # traces[problem][config] = [(iter, theta, fac), ...]
    for f in trace_files:
        # Parse filename: theta_trace_<problem>_<config>.csv
        base = os.path.basename(f).replace("theta_trace_", "").replace(".csv", "")
        # Find the config suffix.
        for c in configs:
            if base.endswith("_" + c):
                prob = base[:-(len(c)+1)]
                with open(f) as fh:
                    reader = csv.DictReader(fh)
                    data = []
                    for row in reader:
                        data.append((int(row['iter']),
                                     float(row['theta']),
                                     int(row['factorizations'])))
                    traces[prob][c] = data
                break

    if not traces:
        print("Could not parse theta trace files")
        return

    # Select a subset of interesting problems.
    interesting = [p for p in problems if p in traces and len(traces[p]) > 1]
    if len(interesting) > 12:
        interesting = interesting[:12]

    n_plots = len(interesting)
    if n_plots == 0:
        return

    cols = min(4, n_plots)
    plot_rows = (n_plots + cols - 1) // cols

    # Plot 1: theta vs iteration.
    fig, axes = plt.subplots(plot_rows, cols, figsize=(4*cols, 3*plot_rows))
    if plot_rows == 1 and cols == 1:
        axes = [[axes]]
    elif plot_rows == 1:
        axes = [axes]
    elif cols == 1:
        axes = [[ax] for ax in axes]

    for idx, prob in enumerate(interesting):
        r, c_idx = idx // cols, idx % cols
        ax = axes[r][c_idx]
        for i, cfg in enumerate(configs):
            if cfg in traces[prob]:
                data = traces[prob][cfg]
                iters = [d[0] for d in data]
                thetas = [abs(d[1]) + 1e-20 for d in data]  # abs for log
                ax.semilogy(iters, thetas, label=cfg,
                           color=colors[i % len(colors)],
                           linestyle=linestyles[i % len(linestyles)],
                           linewidth=1, alpha=0.8)
        ax.set_title(prob, fontsize=9)
        ax.set_xlabel('Iteration', fontsize=7)
        ax.set_ylabel('|theta|', fontsize=7)
        ax.tick_params(labelsize=6)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(left=0)

    # Hide empty axes.
    for idx in range(n_plots, plot_rows * cols):
        r, c_idx = idx // cols, idx % cols
        axes[r][c_idx].set_visible(False)

    axes[0][-1].legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize=6)
    fig.suptitle('Theta Convergence by Problem and Config', fontsize=12)
    fig.tight_layout()
    fig.savefig('theta_convergence.png', dpi=150, bbox_inches='tight')
    print("Wrote theta_convergence.png")

    # Plot 2: theta vs factorizations.
    fig2, axes2 = plt.subplots(plot_rows, cols, figsize=(4*cols, 3*plot_rows))
    if plot_rows == 1 and cols == 1:
        axes2 = [[axes2]]
    elif plot_rows == 1:
        axes2 = [axes2]
    elif cols == 1:
        axes2 = [[ax] for ax in axes2]

    for idx, prob in enumerate(interesting):
        r, c_idx = idx // cols, idx % cols
        ax = axes2[r][c_idx]
        for i, cfg in enumerate(configs):
            if cfg in traces[prob]:
                data = traces[prob][cfg]
                facs = [d[2] for d in data]
                thetas = [abs(d[1]) + 1e-20 for d in data]
                ax.semilogy(facs, thetas, label=cfg,
                           color=colors[i % len(colors)],
                           linestyle=linestyles[i % len(linestyles)],
                           linewidth=1, alpha=0.8)
        ax.set_title(prob, fontsize=9)
        ax.set_xlabel('Factorizations', fontsize=7)
        ax.set_ylabel('|theta|', fontsize=7)
        ax.tick_params(labelsize=6)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(left=0)

    for idx in range(n_plots, plot_rows * cols):
        r, c_idx = idx // cols, idx % cols
        axes2[r][c_idx].set_visible(False)

    axes2[0][-1].legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize=6)
    fig2.suptitle('Theta Convergence vs Factorizations', fontsize=12)
    fig2.tight_layout()
    fig2.savefig('theta_vs_fac.png', dpi=150, bbox_inches='tight')
    print("Wrote theta_vs_fac.png")

def print_ascii_tables(rows):
    """Print ASCII summary tables."""
    configs = sorted(set(r['config'] for r in rows))
    problems = sorted(set(r['problem'] for r in rows))

    print(f"\n=== SCATTER: fac / log10(stat) ===")
    print(f"{'Problem':<16}", end="")
    for c in configs:
        print(f" {c[:14]:>14}", end="")
    print()
    print("-" * (16 + 15 * len(configs)))
    for p in problems:
        print(f"{p:<16}", end="")
        for c in configs:
            match = [r for r in rows if r['problem'] == p and r['config'] == c]
            if match:
                r = match[0]
                s = r['stationarity']
                f = r['factorizations']
                if s > 0:
                    print(f" {f:3d}/{math.log10(s):5.1f}    ", end="")
                else:
                    print(f" {f:3d}/  nan    ", end="")
            else:
                print(f" {'---':>14}", end="")
        print()

    # Summary
    print(f"\n=== SUMMARY ===")
    print(f"{'Config':<20} {'ok<1e-4':>8} {'ok<1e-8':>8} {'avg_fac':>8} {'avg_ms':>8}")
    print("-" * 55)
    for c in configs:
        crows = [r for r in rows if r['config'] == c]
        n = len(crows)
        ok4 = sum(1 for r in crows if r['stationarity'] >= 0 and r['stationarity'] < 1e-4)
        ok8 = sum(1 for r in crows if r['stationarity'] >= 0 and r['stationarity'] < 1e-8)
        af = sum(r['factorizations'] for r in crows) / max(n, 1)
        am = sum(r['time_ms'] for r in crows) / max(n, 1)
        print(f"{c:<20} {ok4:>4}/{n:<3} {ok8:>4}/{n:<3} {af:>8.1f} {am:>8.1f}")

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "eval_thetacontr.csv"
    rows = load_csv(path)
    print(f"Loaded {len(rows)} results from {path}")

    print_ascii_tables(rows)

    if HAS_MPL:
        plot_scatter(rows)
        plot_performance_profile(rows)
        plot_performance_profile_time(rows)
        plot_per_problem(rows)
        plot_theta_convergence(rows)
    else:
        print("\nInstall matplotlib for PNG plots: pip install matplotlib")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def load_rows(csv_path: Path):
    rows = []
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                {
                    "family": row.get("family", "band"),
                    "n": int(row["n"]),
                    "m": int(row["m"]),
                    "nnz": int(row["nnz"]),
                    "tree_width": int(row["tree_width"]),
                    "speedup_qr_over_tree": float(row["speedup_qr_over_tree"]),
                    "speedup_dense_over_tree": float(row["speedup_dense_over_tree"]),
                }
            )
    return rows


def plot_metric(rows, x_key, x_label, out_png: Path):
    plt.figure(figsize=(7.2, 4.8))
    for family, marker in [("band", "o"), ("star", "s")]:
        subset = [r for r in rows if r["family"] == family]
        if not subset:
            continue
        x = [r[x_key] for r in subset]
        y_qr = [r["speedup_qr_over_tree"] for r in subset]
        y_dense = [r["speedup_dense_over_tree"] for r in subset]
        plt.scatter(
            x,
            y_qr,
            s=32,
            alpha=0.85,
            edgecolors="black",
            linewidths=0.3,
            marker=marker,
            label=f"SparseQR / Tree ({family})",
        )
        plt.scatter(
            x,
            y_dense,
            s=32,
            alpha=0.85,
            edgecolors="black",
            linewidths=0.3,
            marker="^" if marker == "o" else "D",
            label=f"Dense / Tree ({family})",
        )
    plt.axhline(1.0, color="red", linestyle="--", linewidth=1.0, label="Parity")
    plt.xlabel(x_label)
    plt.ylabel("Speedup (baseline time / Tree time)")
    plt.title(f"Tree Solver Speedup vs {x_label}")
    plt.grid(alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def main():
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("conex/test/least_squares_speedup.csv")
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("conex/test")
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = load_rows(csv_path)
    if not rows:
        raise SystemExit("No rows found in CSV.")

    plot_metric(rows, "n", "A columns (n)", out_dir / "speedup_vs_n.png")
    plot_metric(rows, "nnz", "nnz(A)", out_dir / "speedup_vs_nnz.png")
    plot_metric(rows, "tree_width", "Tree Width", out_dir / "speedup_vs_tree_width.png")
    print(f"Wrote plots to: {out_dir}")


if __name__ == "__main__":
    main()

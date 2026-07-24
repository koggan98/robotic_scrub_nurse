#!/usr/bin/env python3
"""Offline analysis + plots for a handover/reclaim evaluation session.

Reads the cycles.csv produced by eval_handover_reclaim.py and generates
thesis-ready figures and summary tables. Runs OUTSIDE the ROS env (e.g. on the
dev laptop) -- it only needs pandas + matplotlib, which are not installed in the
robot's ROS Python environment.

    pip install pandas matplotlib
    python3 ros_unrelated_scripts/analyze_eval.py ~/scrub_nurse_eval/20260724_eval
    python3 ros_unrelated_scripts/analyze_eval.py path/to/cycles.csv

Outputs (into <session>/plots/):
    duration_hist.png        total-duration histogram per cycle type
    subphase_box.png         box-plot of sub-phase durations
    success_by_tool.png      success/failure bar chart per tool class
    timeline.png             cumulative completed cycles over time
    summary_table.md / .tex  mean+/-std / median / p95 / success-rate table
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


SUBPHASES = {
    "d_pick": "pick",
    "d_gesture_wait": "gesture wait",
    "d_move_to_hand": "move to hand",
    "d_present_to_taken": "present->taken",
    "d_transport": "transport",
    "d_place": "place",
}


def _load(path: Path):
    import pandas as pd

    if path.is_dir():
        csv_path = path / "cycles.csv"
        session_dir = path
    else:
        csv_path = path
        session_dir = path.parent
    if not csv_path.exists():
        sys.exit(f"cycles.csv not found at: {csv_path}")
    df = pd.read_csv(csv_path)
    if df.empty:
        sys.exit("cycles.csv has no rows -- nothing to analyse.")
    return df, session_dir


def _fmt_stats(series):
    import pandas as pd

    s = pd.to_numeric(series, errors="coerce").dropna()
    if s.empty:
        return None
    return {
        "n": int(s.size), "mean": s.mean(), "std": s.std(ddof=0),
        "median": s.median(), "p95": s.quantile(0.95),
        "min": s.min(), "max": s.max(),
    }


def plot_duration_hist(df, out):
    import matplotlib.pyplot as plt
    import pandas as pd

    types = [t for t in ("handover", "reclaim") if (df["type"] == t).any()]
    if not types:
        return
    fig, axes = plt.subplots(1, len(types), figsize=(6 * len(types), 4), squeeze=False)
    for ax, t in zip(axes[0], types):
        vals = pd.to_numeric(
            df[(df["type"] == t) & (df["outcome"] == "success")]["d_total"],
            errors="coerce").dropna()
        if not vals.empty:
            ax.hist(vals, bins=min(20, max(5, vals.size)),
                    color="tab:blue", edgecolor="white", alpha=0.85)
            ax.axvline(vals.median(), color="tab:red", linestyle="--",
                       label=f"median {vals.median():.1f}s")
            ax.legend()
        ax.set_title(f"{t} total duration (successful)")
        ax.set_xlabel("duration [s]")
        ax.set_ylabel("count")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "duration_hist.png", dpi=150)
    plt.close(fig)


def plot_subphase_box(df, out):
    import matplotlib.pyplot as plt
    import pandas as pd

    data, labels = [], []
    for col, label in SUBPHASES.items():
        if col not in df.columns:
            continue
        vals = pd.to_numeric(df[df["outcome"] == "success"][col], errors="coerce").dropna()
        if not vals.empty:
            data.append(vals.values)
            labels.append(f"{label}\n(n={vals.size})")
    if not data:
        return
    fig, ax = plt.subplots(figsize=(1.6 * len(data) + 2, 4.5))
    ax.boxplot(data, labels=labels, showmeans=True)
    ax.set_title("Sub-phase durations (successful cycles)")
    ax.set_ylabel("duration [s]")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "subphase_box.png", dpi=150)
    plt.close(fig)


def plot_success_by_tool(df, out):
    import matplotlib.pyplot as plt

    sub = df[df["tool_class"].notna() & (df["tool_class"] != "")]
    if sub.empty:
        return
    tools = sorted(sub["tool_class"].unique())
    succ = [((sub["tool_class"] == t) & (sub["outcome"] == "success")).sum() for t in tools]
    fail = [((sub["tool_class"] == t) & (sub["outcome"] == "failure")).sum() for t in tools]
    x = range(len(tools))
    fig, ax = plt.subplots(figsize=(1.1 * len(tools) + 3, 4.5))
    ax.bar(x, succ, label="success", color="tab:green")
    ax.bar(x, fail, bottom=succ, label="failure", color="tab:red")
    ax.set_xticks(list(x))
    ax.set_xticklabels(tools, rotation=30, ha="right")
    ax.set_title("Outcome per tool class")
    ax.set_ylabel("cycles")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "success_by_tool.png", dpi=150)
    plt.close(fig)


def plot_timeline(df, out):
    import matplotlib.pyplot as plt
    import pandas as pd

    t_end = pd.to_numeric(df["t_end_rel"], errors="coerce")
    order = t_end.argsort()
    t = t_end.iloc[order].values
    ok = (df["outcome"].iloc[order] == "success").cumsum().values
    fail = (df["outcome"].iloc[order] == "failure").cumsum().values
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, ok, label="cumulative success", color="tab:green")
    ax.plot(t, fail, label="cumulative failure", color="tab:red")
    ax.set_title("Cumulative cycles over time")
    ax.set_xlabel("time since start [s]")
    ax.set_ylabel("count")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "timeline.png", dpi=150)
    plt.close(fig)


def write_tables(df, out):
    rows = []
    for t in ("handover", "reclaim"):
        sub = df[df["type"] == t]
        if sub.empty:
            continue
        n = len(sub)
        succ = int((sub["outcome"] == "success").sum())
        st = _fmt_stats(sub[sub["outcome"] == "success"]["d_total"])
        rows.append({
            "type": t, "n": n, "success": succ,
            "success_rate_pct": round(100 * succ / n, 1),
            "d_total_mean": round(st["mean"], 2) if st else "",
            "d_total_std": round(st["std"], 2) if st else "",
            "d_total_median": round(st["median"], 2) if st else "",
            "d_total_p95": round(st["p95"], 2) if st else "",
        })

    header = ["type", "n", "success", "success_rate_pct",
              "d_total_mean", "d_total_std", "d_total_median", "d_total_p95"]

    md = ["| " + " | ".join(header) + " |",
          "|" + "|".join(["---"] * len(header)) + "|"]
    for r in rows:
        md.append("| " + " | ".join(str(r[h]) for h in header) + " |")
    (out / "summary_table.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    tex = [r"\begin{tabular}{l" + "r" * (len(header) - 1) + "}", r"\hline",
           " & ".join(h.replace("_", r"\_") for h in header) + r" \\", r"\hline"]
    for r in rows:
        tex.append(" & ".join(str(r[h]) for h in header) + r" \\")
    tex += [r"\hline", r"\end{tabular}"]
    (out / "summary_table.tex").write_text("\n".join(tex) + "\n", encoding="utf-8")

    print("\n".join(md))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="Session directory or path to cycles.csv")
    args = ap.parse_args()

    try:
        import matplotlib  # noqa: F401
        import pandas  # noqa: F401
    except ImportError:
        sys.exit("This script needs pandas + matplotlib. Install with:\n"
                 "    pip install pandas matplotlib")
    import matplotlib
    matplotlib.use("Agg")

    df, session_dir = _load(Path(args.session).expanduser())
    out = session_dir / "plots"
    out.mkdir(parents=True, exist_ok=True)

    plot_duration_hist(df, out)
    plot_subphase_box(df, out)
    plot_success_by_tool(df, out)
    plot_timeline(df, out)
    write_tables(df, out)

    print(f"\nWrote figures + tables to: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

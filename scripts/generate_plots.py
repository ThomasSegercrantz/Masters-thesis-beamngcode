#!/usr/bin/env python3
"""
Plot BeamNG ACC scenario runs together.

Usage:
    python scripts/plot_scenario_runs.py 21_baseline_follow
    python scripts/plot_scenario_runs.py 22_hard_brake --outdir results/plots
    python scripts/plot_scenario_runs.py 23_lead_slowdown --rawdir results/raw

What it does:
- Finds all raw CSV files in results/raw matching the given scenario prefix
- Example prefix: 21_baseline_follow
- Loads all matching runs
- Plots raw time-series together
- Ego traces are blue, lead traces are red
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List

import matplotlib.pyplot as plt
import pandas as pd


def load_run(csv_path: Path) -> pd.DataFrame:
    """Load a BeamNG raw CSV, skipping metadata/comment lines."""
    df = pd.read_csv(csv_path, comment="#")
    if "time_s" not in df.columns:
        raise ValueError(f"{csv_path} does not look like a valid raw run CSV.")
    return df


def kph(series: pd.Series) -> pd.Series:
    return series * 3.6


def ensure_outdir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def find_matching_csvs(rawdir: Path, scenario_prefix: str) -> List[Path]:
    """Find all CSV files matching e.g. 21_baseline_follow_*.csv."""
    matches = sorted(rawdir.glob(f"{scenario_prefix}_*.csv"))
    if not matches:
        raise FileNotFoundError(
            f"No CSV files found for prefix '{scenario_prefix}' in '{rawdir}'."
        )
    return matches


def short_name_from_path(path: Path, scenario_prefix: str) -> str:
    """Use only timestamp part in legend if possible."""
    stem = path.stem
    prefix = f"{scenario_prefix}_"
    if stem.startswith(prefix):
        return stem[len(prefix):]
    return stem

def format_scenario_name(scenario_prefix: str) -> str:
    # Convert '21_baseline_follow' -> 'Baseline Follow'
    # Remove leading number + underscore
    parts = scenario_prefix.split("_", 1)
    if len(parts) > 1 and parts[0].isdigit():
        name = parts[1]
    else:
        name = scenario_prefix

    # Replace underscores and capitalize nicely
    return name.replace("_", " ").title()


def plot_speed(runs: List[pd.DataFrame], names: List[str], outdir: Path, display_name: str) -> None:
    plt.figure(figsize=(11, 6))
    linestyles = ["-", "--", ":"]
    for i, (df, name) in enumerate(zip(runs, names)):
        ls = linestyles[i % len(linestyles)]
        plt.plot(
            df["time_s"], kph(df["ego_speed_mps"]),
            linestyle=ls, linewidth=2, alpha=0.9,
            color="blue", label=f"Ego {name}"
        )
        plt.plot(
            df["time_s"], kph(df["lead_speed_mps"]),
            linestyle=ls, linewidth=2, alpha=0.9,
            color="red", label=f"Lead {name}"
        )

    plt.xlabel("Time [s]")
    plt.ylabel("Speed [km/h]")
    plt.title(f"{display_name}: Ego and Lead Speed vs Time")
    plt.grid(True, alpha=0.3)
    plt.legend(ncol=2, fontsize=9)
    plt.tight_layout()
    plt.savefig(outdir / f"{display_name}_speed_vs_time.png", dpi=300)
    plt.close()


def plot_gap(runs: List[pd.DataFrame], names: List[str], outdir: Path, display_name: str) -> None:
    plt.figure(figsize=(11, 6))
    linestyles = ["-", "--", ":"]
    for i, (df, name) in enumerate(zip(runs, names)):
        ls = linestyles[i % len(linestyles)]
        plt.plot(
            df["time_s"], df["gap_m"],
            linestyle=ls, linewidth=2, alpha=0.9,
            color="blue", label=f"Run {name}"
        )

    plt.xlabel("Time [s]")
    plt.ylabel("Gap [m]")
    plt.title(f"{display_name}: Gap Distance vs Time")
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=9)
    plt.tight_layout()
    plt.savefig(outdir / f"{display_name}_gap_vs_time.png", dpi=300)
    plt.close()


def plot_relative_speed(runs: List[pd.DataFrame], names: List[str], outdir: Path, display_name: str) -> None:
    plt.figure(figsize=(11, 6))
    linestyles = ["-", "--", ":"]
    for i, (df, name) in enumerate(zip(runs, names)):
        ls = linestyles[i % len(linestyles)]
        plt.plot(
            df["time_s"], df["relative_speed_mps"],
            linestyle=ls, linewidth=2, alpha=0.9,
            color="purple", label=f"Run {name}"
        )

    plt.axhline(0.0, linewidth=1, color="black")
    plt.xlabel("Time [s]")
    plt.ylabel("Relative Speed [m/s]")
    plt.title(f"{display_name}: Relative Speed vs Time")
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=9)
    plt.tight_layout()
    plt.savefig(outdir / f"{display_name}_relative_speed_vs_time.png", dpi=300)
    plt.close()


def plot_acceleration(runs: List[pd.DataFrame], names: List[str], outdir: Path, display_name: str) -> None:
    plt.figure(figsize=(11, 6))
    linestyles = ["-", "--", ":"]
    for i, (df, name) in enumerate(zip(runs, names)):
        ls = linestyles[i % len(linestyles)]
        plt.plot(
            df["time_s"], df["ego_accel_mps2"],
            linestyle=ls, linewidth=2, alpha=0.9,
            color="blue", label=f"Ego {name}"
        )
        plt.plot(
            df["time_s"], df["lead_accel_mps2"],
            linestyle=ls, linewidth=2, alpha=0.9,
            color="red", label=f"Lead {name}"
        )

    plt.axhline(0.0, linewidth=1, color="black")
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s²]")
    plt.title(f"{display_name}: Ego and Lead Acceleration vs Time")
    plt.grid(True, alpha=0.3)
    plt.legend(ncol=2, fontsize=9)
    plt.tight_layout()
    plt.savefig(outdir / f"{display_name}_acceleration_vs_time.png", dpi=300)
    plt.close()


def plot_headway(runs: List[pd.DataFrame], names: List[str], outdir: Path, display_name: str) -> None:
    plt.figure(figsize=(11, 6))
    linestyles = ["-", "--", ":"]
    for i, (df, name) in enumerate(zip(runs, names)):
        ls = linestyles[i % len(linestyles)]
        ego_speed = df["ego_speed_mps"].copy()
        headway_s = df["gap_m"] / ego_speed.where(ego_speed > 0.1)
        plt.plot(
            df["time_s"], headway_s,
            linestyle=ls, linewidth=2, alpha=0.9,
            color="blue", label=f"Run {name}"
        )

    plt.xlabel("Time [s]")
    plt.ylabel("Headway [s]")
    plt.title(f"{display_name}: Headway vs Time")
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=9)
    plt.tight_layout()
    plt.savefig(outdir / f"{display_name}_headway_vs_time.png", dpi=300)
    plt.close()


def plot_trajectory(runs: List[pd.DataFrame], names: List[str], outdir: Path, display_name: str) -> None:
    plt.figure(figsize=(8, 8))
    linestyles = ["-", "--", ":"]
    for i, (df, name) in enumerate(zip(runs, names)):
        ls = linestyles[i % len(linestyles)]
        plt.plot(
            df["ego_pos_x"], df["ego_pos_y"],
            linestyle=ls, linewidth=2, alpha=0.9,
            color="blue", label=f"Ego {name}"
        )
        plt.plot(
            df["lead_pos_x"], df["lead_pos_y"],
            linestyle=ls, linewidth=2, alpha=0.9,
            color="red", label=f"Lead {name}"
        )

    plt.xlabel("X Position [m]")
    plt.ylabel("Y Position [m]")
    plt.title(f"{display_name}: Vehicle Trajectories")
    plt.grid(True, alpha=0.3)
    plt.axis("equal")
    plt.legend(ncol=2, fontsize=9)
    plt.tight_layout()
    plt.savefig(outdir / f"{display_name}_trajectory_xy.png", dpi=300)
    plt.close()


def plot_controls_if_present(runs: List[pd.DataFrame], names: List[str], outdir: Path, display_name: str) -> None:
    control_cols = ["ego_throttle", "ego_brake", "ego_steering"]
    present = [c for c in control_cols if all(c in df.columns for df in runs)]
    if not present:
        return

    plt.figure(figsize=(11, 7))
    linestyles = ["-", "--", ":"]

    for i, (df, name) in enumerate(zip(runs, names)):
        ls = linestyles[i % len(linestyles)]
        if "ego_throttle" in present:
            plt.plot(
                df["time_s"], df["ego_throttle"],
                linestyle=ls, linewidth=1.8, alpha=0.8,
                color="green", label=f"Throttle {name}"
            )
        if "ego_brake" in present:
            plt.plot(
                df["time_s"], df["ego_brake"],
                linestyle=ls, linewidth=1.8, alpha=0.8,
                color="orange", label=f"Brake {name}"
            )
        if "ego_steering" in present:
            plt.plot(
                df["time_s"], df["ego_steering"],
                linestyle=ls, linewidth=1.8, alpha=0.8,
                color="black", label=f"Steering {name}"
            )

    plt.xlabel("Time [s]")
    plt.ylabel("Control Signal [-]")
    plt.title(f"{display_name}: Logged Ego Control Signals")
    plt.grid(True, alpha=0.3)
    plt.legend(ncol=3, fontsize=8)
    plt.tight_layout()
    plt.savefig(outdir / f"{display_name}_ego_controls.png", dpi=300)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot all raw CSV runs for one BeamNG scenario prefix."
    )
    parser.add_argument(
        "scenario_prefix",
        help="Scenario prefix, e.g. 21_baseline_follow"
    )
    parser.add_argument(
        "--rawdir",
        type=Path,
        default=Path("results/raw"),
        help="Directory containing raw CSV files (default: results/raw)"
    )
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path("results/plots"),
        help="Base output directory for plots (default: results/plots)"
    )
    args = parser.parse_args()

    rawdir = args.rawdir
    scenario_prefix = args.scenario_prefix
    outdir = ensure_outdir(args.outdir / scenario_prefix)

    csv_files = find_matching_csvs(rawdir, scenario_prefix)
    runs = [load_run(path) for path in csv_files]
    names = [short_name_from_path(path, scenario_prefix) for path in csv_files]
    display_name = format_scenario_name(scenario_prefix)

    print(f"Found {len(csv_files)} matching run(s):")
    for path in csv_files:
        print(f"  - {path}")

    plot_speed(runs, names, outdir, display_name)
    plot_gap(runs, names, outdir, display_name)
    plot_relative_speed(runs, names, outdir, display_name)
    plot_acceleration(runs, names, outdir, display_name)
    plot_headway(runs, names, outdir, display_name)
    plot_trajectory(runs, names, outdir, display_name)
    plot_controls_if_present(runs, names, outdir, display_name)

    print(f"\nSaved plots to: {outdir.resolve()}")


if __name__ == "__main__":
    main()
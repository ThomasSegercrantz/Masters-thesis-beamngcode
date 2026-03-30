#!/usr/bin/env python3
"""
BeamNG.tech ACC experiment analysis
===================================

Reads all telemetry CSV logs in:
  results/raw/*.csv

Produces by default:
  1) results/summary/summary_metrics.csv              -> one combined summary for all runs
  2) results/summary/<run_name>_summary.csv          -> one summary CSV per run

Optional:
  3) results/summary/plots/<run_name>.png            -> one plot per run (if --make_plots)

Usage examples
--------------
From repo root:

  python scripts/analyze_runs.py

Optional:
  python scripts/analyze_runs.py --make_plots
  python scripts/analyze_runs.py --raw_dir results/raw --summary_dir results/summary
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import pandas as pd


def parse_metadata(csv_path: Path) -> dict:
    """
    Parses leading '# key: value' lines from the CSV into a dict.
    """
    meta = {}
    with csv_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("#"):
                break
            line = line[1:].strip()
            if ":" in line:
                k, v = line.split(":", 1)
                meta[k.strip()] = v.strip()
    return meta


def rms(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(x * x)))


def safe_stem(path: Path) -> str:
    """
    Sanitized stem for output filenames.
    """
    s = path.stem
    for ch in [" ", ":", "/", "\\"]:
        s = s.replace(ch, "_")
    return s


def scenario_prefix_from_name(name: str) -> str:
    """
    Example:
      22_hard_brake_20260323_131250 -> 22_hard_brake
    """
    parts = name.split("_")
    if len(parts) >= 3:
        return "_".join(parts[:3])
    return name


def should_trim_post_stop_restart(csv_path: Path) -> bool:
    """
    Apply trimming only to scenarios where reverse motion after stopping
    contaminates the performance metrics.
    """
    prefix = scenario_prefix_from_name(csv_path.stem)
    return prefix in {"22_hard_brake", "25_gradual_stop"}


def find_post_stop_restart_cutoff(
    df: pd.DataFrame,
    move_threshold_mps: float = 5.0,
    stop_threshold_mps: float = 0.5,
    restart_threshold_mps: float = 0.5,
    stop_consecutive_samples: int = 3,
) -> tuple[int | None, float | None]:
    """
    Finds the first frame where the ego starts moving again after having:
      1) moved initially
      2) come to a stop

    This is intended to exclude simulator reverse motion after a collision or
    after the vehicle has already come to rest.

    Returns:
      cutoff_idx, cutoff_time_s

    If no cutoff is found, returns (None, None).

    Logic:
    - detect that the ego has been moving (> move_threshold_mps)
    - find the first sustained stop (< stop_threshold_mps for N consecutive samples)
    - after that stop, find the first frame where speed rises again (> restart_threshold_mps)
    - use that first restart frame as cutoff
    """
    ego_speed = df["ego_speed_mps"].to_numpy(dtype=float)
    time_s = df["time_s"].to_numpy(dtype=float)

    # Must have moved at some point first
    moving_idx = np.where(ego_speed > move_threshold_mps)[0]
    if moving_idx.size == 0:
        return None, None

    start_search_idx = moving_idx[0]

    # Find first sustained stop after initial motion
    stop_start_idx = None
    consec = 0
    for i in range(start_search_idx, len(ego_speed)):
        if ego_speed[i] < stop_threshold_mps:
            consec += 1
            if consec >= stop_consecutive_samples:
                stop_start_idx = i - stop_consecutive_samples + 1
                break
        else:
            consec = 0

    if stop_start_idx is None:
        return None, None

    # Find first restart after that stop
    for i in range(stop_start_idx + stop_consecutive_samples, len(ego_speed)):
        if ego_speed[i] > restart_threshold_mps:
            return i, float(time_s[i])

    return None, None


def trim_evaluation_window(csv_path: Path, df: pd.DataFrame) -> tuple[pd.DataFrame, bool, float | None]:
    """
    Returns:
      df_eval, trim_applied, cutoff_time_s

    For hard brake and gradual stop scenarios:
      trim at the first frame where ego starts moving again after first stopping.

    For all other scenarios:
      return full dataframe unchanged.
    """
    if not should_trim_post_stop_restart(csv_path):
        return df.copy(), False, None

    cutoff_idx, cutoff_time_s = find_post_stop_restart_cutoff(df)
    if cutoff_idx is None:
        return df.copy(), False, None

    # Exclude the first restart frame itself from evaluation
    df_eval = df.iloc[:cutoff_idx].copy()
    return df_eval, True, cutoff_time_s


def compute_metrics(
    df: pd.DataFrame,
    meta: dict,
    collision_gap_m: float,
    raw_duration_s: float | None = None,
    trim_applied: bool = False,
    cutoff_time_s: float | None = None,
) -> dict:
    out = {}
    out["file"] = meta.get("File", "")
    out["script"] = meta.get("Script", "")
    out["date"] = meta.get("Date", "")
    out["description"] = meta.get("Description", "")

    acc_target_kph = None
    lead_target_kph = None
    try:
        if "ACC target (kph)" in meta:
            acc_target_kph = float(meta["ACC target (kph)"])
        if "Lead target (kph)" in meta:
            lead_target_kph = float(meta["Lead target (kph)"])
    except Exception:
        pass

    out["acc_target_kph"] = acc_target_kph
    out["lead_target_kph"] = lead_target_kph

    out["raw_duration_s"] = float(raw_duration_s) if raw_duration_s is not None else float(df["time_s"].max())
    out["eval_duration_s"] = float(df["time_s"].max())
    out["duration_s"] = out["eval_duration_s"]  # keep legacy column name
    out["trim_applied"] = bool(trim_applied)
    out["analysis_cutoff_time_s"] = float(cutoff_time_s) if cutoff_time_s is not None else float("nan")

    ego_kph = df["ego_speed_mps"] * 3.6
    lead_kph = df["lead_speed_mps"] * 3.6

    out["ego_speed_max_kph"] = float(ego_kph.max())
    out["ego_speed_mean_kph"] = float(ego_kph.mean())
    out["ego_speed_min_kph"] = float(ego_kph.min())

    out["lead_speed_max_kph"] = float(lead_kph.max())
    out["lead_speed_mean_kph"] = float(lead_kph.mean())
    out["lead_speed_min_kph"] = float(lead_kph.min())

    gap = df["gap_m"]
    out["gap_min_m"] = float(gap.min())
    out["gap_mean_m"] = float(gap.mean())
    out["gap_max_m"] = float(gap.max())

    moving = df["ego_speed_mps"] > 5.0
    rel_m = df.loc[moving, "relative_speed_mps"].to_numpy(dtype=float)
    out["rel_speed_rms_mps"] = rms(rel_m)
    out["rel_speed_rms_kph"] = (
        out["rel_speed_rms_mps"] * 3.6
        if not math.isnan(out["rel_speed_rms_mps"])
        else float("nan")
    )

    ego_mps = df["ego_speed_mps"].to_numpy(dtype=float)
    gap_m = df["gap_m"].to_numpy(dtype=float)
    headway = np.full_like(ego_mps, np.nan, dtype=float)
    idx = ego_mps > 5.0
    headway[idx] = gap_m[idx] / np.maximum(ego_mps[idx], 1e-6)
    out["headway_mean_s"] = float(np.nanmean(headway))
    out["headway_min_s"] = float(np.nanmin(headway))
    out["headway_max_s"] = float(np.nanmax(headway))

    a = df["ego_accel_mps2"].to_numpy(dtype=float)
    out["ego_accel_peak_mps2"] = float(np.nanmax(np.abs(a)))
    out["ego_max_decel_mps2"] = float(np.nanmin(a))

    t = df["time_s"].to_numpy(dtype=float)
    if len(t) >= 2:
        dt = np.diff(t)
        da = np.diff(a)
        jerk = np.where(dt > 1e-6, da / dt, np.nan)
        out["ego_jerk_peak_mps3"] = float(np.nanmax(np.abs(jerk)))
    else:
        out["ego_jerk_peak_mps3"] = float("nan")

    rel_mps = df["relative_speed_mps"].to_numpy(dtype=float)
    ttc = np.full_like(rel_mps, np.nan, dtype=float)
    closing = rel_mps > 0.5
    if np.any(closing):
        ttc[closing] = gap_m[closing] / np.maximum(rel_mps[closing], 1e-6)
        out["ttc_proxy_min_s"] = float(np.nanmin(ttc))
    else:
        out["ttc_proxy_min_s"] = float("nan")

    out["collision_flag"] = bool(out["gap_min_m"] < collision_gap_m)

    if acc_target_kph is not None:
        err = ego_kph - acc_target_kph
        out["ego_overshoot_kph"] = float(np.max(err))

        t_end = float(df["time_s"].max())
        tail = df[df["time_s"] >= max(0.0, t_end - 10.0)]
        if len(tail) > 5:
            tail_err = (tail["ego_speed_mps"] * 3.6) - acc_target_kph
            out["ego_steady_state_abs_error_kph"] = float(np.mean(np.abs(tail_err)))
        else:
            out["ego_steady_state_abs_error_kph"] = float("nan")
    else:
        out["ego_overshoot_kph"] = float("nan")
        out["ego_steady_state_abs_error_kph"] = float("nan")

    out["ego_stop_time_s"] = float("nan")
    was_moving = np.where(ego_mps > 5.0)[0]
    if was_moving.size > 0:
        start_i = was_moving[0]
        stopped = np.where((np.arange(len(ego_mps)) > start_i) & (ego_mps < 0.5))[0]
        if stopped.size > 0:
            out["ego_stop_time_s"] = float(t[stopped[0]])

    return out


def ordered_summary_df(rows: list[dict]) -> pd.DataFrame:
    out_df = pd.DataFrame(rows)

    preferred = [
        "file", "script", "date", "description",
        "acc_target_kph", "lead_target_kph",
        "raw_duration_s", "eval_duration_s", "duration_s",
        "trim_applied", "analysis_cutoff_time_s",
        "ego_speed_mean_kph", "ego_speed_max_kph", "ego_speed_min_kph",
        "lead_speed_mean_kph", "lead_speed_max_kph", "lead_speed_min_kph",
        "gap_mean_m", "gap_min_m", "gap_max_m",
        "headway_mean_s", "headway_min_s", "headway_max_s",
        "rel_speed_rms_mps", "rel_speed_rms_kph",
        "ego_accel_peak_mps2", "ego_max_decel_mps2", "ego_jerk_peak_mps3",
        "ttc_proxy_min_s",
        "ego_overshoot_kph", "ego_steady_state_abs_error_kph",
        "ego_stop_time_s",
        "collision_flag",
    ]
    cols = [c for c in preferred if c in out_df.columns] + [c for c in out_df.columns if c not in preferred]
    return out_df[cols]


def write_single_run_summary(summary_path: Path, metrics: dict, collision_gap_m: float) -> None:
    df = ordered_summary_df([metrics])
    with summary_path.open("w", encoding="utf-8", newline="") as f:
        f.write("# BeamNG.tech per-run summary metrics\n")
        f.write("# Generated by analyze_runs.py\n")
        f.write(f"# collision_gap_m = {collision_gap_m}\n")
        df.to_csv(f, index=False)


def write_combined_summary(summary_path: Path, rows: list[dict], collision_gap_m: float) -> None:
    out_df = ordered_summary_df(rows)
    with summary_path.open("w", encoding="utf-8", newline="") as f:
        f.write("# BeamNG.tech experiment summary metrics\n")
        f.write("# Generated by analyze_runs.py\n")
        f.write(f"# collision_gap_m = {collision_gap_m}\n")
        out_df.to_csv(f, index=False)


def make_run_plot(df: pd.DataFrame, metrics: dict, plot_path: Path) -> None:
    import matplotlib.pyplot as plt

    t = df["time_s"]

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    axes[0].plot(t, df["ego_speed_mps"] * 3.6, label="Ego speed (km/h)")
    axes[0].plot(t, df["lead_speed_mps"] * 3.6, label="Lead speed (km/h)")
    if pd.notna(metrics.get("acc_target_kph")):
        axes[0].axhline(metrics["acc_target_kph"], linestyle="--", label="ACC target")
    axes[0].set_ylabel("Speed")
    axes[0].set_title(metrics.get("file", "Run"))
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, df["gap_m"], label="Gap (m)")
    axes[1].set_ylabel("Gap")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(t, df["relative_speed_mps"], label="Relative speed (m/s)")
    axes[2].axhline(0.0, linestyle="--")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Rel speed")
    axes[2].legend()
    axes[2].grid(True)

    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw_dir", default="results/raw", help="Directory containing raw telemetry CSV files")
    ap.add_argument("--summary_dir", default="results/summary", help="Directory to write summary outputs")
    ap.add_argument("--collision_gap_m", type=float, default=5.0, help="Gap threshold (m) for collision flag")
    ap.add_argument(
        "--make_plots",
        action="store_true",
        help="If set, generate one PNG plot per run in results/summary/plots",
    )
    args = ap.parse_args()

    raw_dir = Path(args.raw_dir)
    summary_dir = Path(args.summary_dir)
    summary_dir.mkdir(parents=True, exist_ok=True)

    plots_dir = summary_dir / "plots"
    if args.make_plots:
        plots_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    processed = 0

    for csv_path in sorted(raw_dir.glob("*.csv")):
        meta = parse_metadata(csv_path)
        meta["File"] = csv_path.name

        try:
            df_raw = pd.read_csv(csv_path, comment="#")
        except Exception as e:
            print(f"[WARN] Failed to read {csv_path}: {e}")
            continue

        required = {"time_s", "ego_speed_mps", "ego_accel_mps2", "lead_speed_mps", "gap_m", "relative_speed_mps"}
        missing = required - set(df_raw.columns)
        if missing:
            print(f"[WARN] Skipping {csv_path.name}: missing required columns {missing}")
            continue

        raw_duration_s = float(df_raw["time_s"].max())
        df_eval, trim_applied, cutoff_time_s = trim_evaluation_window(csv_path, df_raw)

        metrics = compute_metrics(
            df_eval,
            meta,
            collision_gap_m=args.collision_gap_m,
            raw_duration_s=raw_duration_s,
            trim_applied=trim_applied,
            cutoff_time_s=cutoff_time_s,
        )
        rows.append(metrics)
        processed += 1

        run_name = safe_stem(csv_path)
        per_run_summary_path = summary_dir / f"{run_name}_summary.csv"
        write_single_run_summary(per_run_summary_path, metrics, args.collision_gap_m)

        if args.make_plots:
            plot_path = plots_dir / f"{run_name}.png"
            try:
                # keep plots based on raw data, as requested
                make_run_plot(df_raw, metrics, plot_path)
            except Exception as e:
                print(f"[WARN] Failed to plot {csv_path.name}: {e}")

    if not rows:
        print("No valid CSV logs found.")
        return

    combined_summary_path = summary_dir / "summary_metrics.csv"
    write_combined_summary(combined_summary_path, rows, args.collision_gap_m)

    print(f"Wrote combined summary: {combined_summary_path}")
    print(f"Wrote per-run summaries: {processed}")
    if args.make_plots:
        print(f"Wrote plots to: {plots_dir}")


if __name__ == "__main__":
    main()
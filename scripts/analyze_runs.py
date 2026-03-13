#!/usr/bin/env python3
"""
BeamNG.tech ACC experiment analysis
===================================

This script reads all telemetry CSV logs in:
  results/raw/*.csv

and produces:
  results/summary/summary_metrics.csv

It is designed to work with the standardized logger you added in common.py
(BeamNGTelemetryLogger). The CSVs are expected to have commented metadata
lines at the top (starting with '#') and a consistent set of columns such as:

  time_s, ego_speed_mps, ego_accel_mps2, lead_speed_mps, lead_accel_mps2,
  gap_m, relative_speed_mps, ego_throttle, ego_brake, ego_steering,
  ego_pos_x, ego_pos_y, ego_pos_z, lead_pos_x, lead_pos_y, lead_pos_z

------------------------------------------------------------
Glossary / metric definitions (used in your scenario table)
------------------------------------------------------------

Speed (m/s, km/h)
  Vehicle forward speed magnitude. (We use the magnitude of velocity provided
  by BeamNG/BeamNGpy telemetry; your logger already stores speed in m/s.)

Relative speed (ego - lead)
  relative_speed_mps = ego_speed_mps - lead_speed_mps
  Positive means the ego is closing in on the lead (ego faster than lead).

Gap (m)
  Euclidean distance between ego and lead positions:
    gap_m = ||ego_pos - lead_pos||
  Note: this is not strictly "bumper-to-bumper" distance, but it is a good
  proxy if vehicles are aligned in-lane.

Headway (s)
  A standard ACC metric: "time gap" to the lead, computed as:
    headway_s = gap_m / max(ego_speed_mps, eps)
  Interpretation:
    - 1.0 s: aggressive following
    - 1.5 s: typical ACC
    - 2.0+ s: conservative following

RMS (Root Mean Square)
  RMS measures magnitude of a signal over time:
    RMS(x) = sqrt(mean(x^2))
  We use it for relative speed stability:
    rel_speed_rms_mps = RMS(relative_speed_mps)
  Lower is better (more stable matching of lead speed).

Acceleration peak (m/s^2)
  Maximum absolute longitudinal acceleration magnitude from the logged
  ego_accel_mps2:
    accel_peak_mps2 = max(abs(ego_accel_mps2))
  We also report max braking (most negative accel):
    max_decel_mps2 = min(ego_accel_mps2)

Jerk peak (m/s^3) (proxy)
  Jerk is the derivative of acceleration. With discrete logs:
    jerk ~= d(accel)/dt ~= Delta(accel) / Delta(t)
  Higher jerk often correlates with lower comfort.

TTC proxy (Time-To-Collision proxy)
  Classic TTC is gap / closing_speed when closing_speed > 0.
  Here we compute:
    ttc_proxy_s = gap_m / max(relative_speed_mps, eps)   (only where rel_speed > 0)
  We report min TTC proxy across the run. Lower values indicate higher risk.
  This is a proxy because it uses Euclidean gap, not exact longitudinal distance.

Collision flag
  A simple boolean indicating a "likely collision" event:
    collision_flag = (min_gap_m < COLLISION_GAP_M)
  Since we don't have bumper geometry, choose a conservative threshold
  (default 5 m). Adjust as needed.

Notes on event-timed metrics (settling time, reaction time, rise time)
  Some scenario-table metrics require knowing the time of an external event
  (e.g., "lead brakes at t=25s"). Your current CSV logs do not yet include
  explicit event markers. This script computes robust "always available"
  metrics from the telemetry alone, and computes a few event-style proxies
  when possible. If you later add an "event" column to the logger, the script
  can be extended to compute those event metrics precisely.

------------------------------------------------------------
Usage
------------------------------------------------------------

From your repo root (where results/ exists):

  python analyze_runs.py

Optional flags:
  --raw_dir results/raw
  --summary_dir results/summary
  --collision_gap_m 5.0
"""
import argparse
from pathlib import Path
import math

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


def compute_metrics(df: pd.DataFrame, meta: dict, collision_gap_m: float) -> dict:
    out = {}
    out["file"] = meta.get("File", "")
    out["script"] = meta.get("Script", "")
    out["date"] = meta.get("Date", "")
    out["description"] = meta.get("Description", "")

    # Targets if present
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

    # Duration
    out["duration_s"] = float(df["time_s"].max())

    ego_kph = df["ego_speed_mps"] * 3.6
    lead_kph = df["lead_speed_mps"] * 3.6

    # Basic speed stats
    out["ego_speed_max_kph"] = float(ego_kph.max())
    out["ego_speed_mean_kph"] = float(ego_kph.mean())
    out["ego_speed_min_kph"] = float(ego_kph.min())
    out["lead_speed_max_kph"] = float(lead_kph.max())
    out["lead_speed_mean_kph"] = float(lead_kph.mean())
    out["lead_speed_min_kph"] = float(lead_kph.min())

    # Gap stats
    gap = df["gap_m"]
    out["gap_min_m"] = float(gap.min())
    out["gap_mean_m"] = float(gap.mean())
    out["gap_max_m"] = float(gap.max())

    # Relative speed RMS (moving only)
    moving = df["ego_speed_mps"] > 5.0  # > 18 km/h
    rel_m = df.loc[moving, "relative_speed_mps"].to_numpy(dtype=float)
    out["rel_speed_rms_mps"] = rms(rel_m)
    out["rel_speed_rms_kph"] = out["rel_speed_rms_mps"] * 3.6 if not math.isnan(out["rel_speed_rms_mps"]) else float("nan")

    # Headway (moving only)
    ego_mps = df["ego_speed_mps"].to_numpy(dtype=float)
    gap_m = df["gap_m"].to_numpy(dtype=float)
    headway = np.full_like(ego_mps, np.nan, dtype=float)
    idx = ego_mps > 5.0
    headway[idx] = gap_m[idx] / np.maximum(ego_mps[idx], 1e-6)
    out["headway_mean_s"] = float(np.nanmean(headway))
    out["headway_min_s"] = float(np.nanmin(headway))
    out["headway_max_s"] = float(np.nanmax(headway))

    # Accel & jerk
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

    # TTC proxy (min) when closing
    rel_mps = df["relative_speed_mps"].to_numpy(dtype=float)
    ttc = np.full_like(rel_mps, np.nan, dtype=float)
    closing = rel_mps > 0.5
    if np.any(closing):
        ttc[closing] = gap_m[closing] / np.maximum(rel_mps[closing], 1e-6)
        out["ttc_proxy_min_s"] = float(np.nanmin(ttc))
    else:
        out["ttc_proxy_min_s"] = float("nan")

    # Collision flag (gap threshold)
    out["collision_flag"] = bool(out["gap_min_m"] < collision_gap_m)

    # Overshoot & steady-state error vs ACC set speed (if known)
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

    # Stop time proxy: first time ego < 0.5 m/s after being > 5 m/s
    out["ego_stop_time_s"] = float("nan")
    was_moving = np.where(ego_mps > 5.0)[0]
    if was_moving.size > 0:
        start_i = was_moving[0]
        stopped = np.where((np.arange(len(ego_mps)) > start_i) & (ego_mps < 0.5))[0]
        if stopped.size > 0:
            out["ego_stop_time_s"] = float(t[stopped[0]])

    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw_dir", default="results/raw", help="Directory containing raw telemetry CSV files")
    ap.add_argument("--summary_dir", default="results/summary", help="Directory to write summary outputs")
    ap.add_argument("--collision_gap_m", type=float, default=5.0, help="Gap threshold (m) for collision flag")
    args = ap.parse_args()

    raw_dir = Path(args.raw_dir)
    summary_dir = Path(args.summary_dir)
    summary_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for csv_path in sorted(raw_dir.glob("*.csv")):
        meta = parse_metadata(csv_path)
        meta["File"] = csv_path.name

        try:
            df = pd.read_csv(csv_path, comment="#")
        except Exception as e:
            print(f"[WARN] Failed to read {csv_path}: {e}")
            continue

        required = {"time_s", "ego_speed_mps", "ego_accel_mps2", "lead_speed_mps", "gap_m", "relative_speed_mps"}
        if not required.issubset(set(df.columns)):
            print(f"[WARN] Skipping {csv_path.name}: missing required columns {required - set(df.columns)}")
            continue

        rows.append(compute_metrics(df, meta, collision_gap_m=args.collision_gap_m))

    if not rows:
        print("No valid CSV logs found.")
        return

    out_df = pd.DataFrame(rows)

    preferred = [
        "file", "script", "date", "description",
        "acc_target_kph", "lead_target_kph",
        "duration_s",
        "ego_speed_mean_kph", "ego_speed_max_kph",
        "lead_speed_mean_kph", "lead_speed_max_kph",
        "gap_mean_m", "gap_min_m",
        "headway_mean_s", "headway_min_s",
        "rel_speed_rms_kph",
        "ego_accel_peak_mps2", "ego_max_decel_mps2", "ego_jerk_peak_mps3",
        "ttc_proxy_min_s",
        "ego_overshoot_kph", "ego_steady_state_abs_error_kph",
        "ego_stop_time_s",
        "collision_flag"
    ]
    cols = [c for c in preferred if c in out_df.columns] + [c for c in out_df.columns if c not in preferred]
    out_df = out_df[cols]

    summary_path = summary_dir / "summary_metrics.csv"
    with summary_path.open("w", encoding="utf-8", newline="") as f:
        f.write("# BeamNG.tech experiment summary metrics\n")
        f.write("# Generated by analyze_runs.py\n")
        f.write(f"# collision_gap_m = {args.collision_gap_m}\n")
        out_df.to_csv(f, index=False)

    print(f"Wrote: {summary_path}\n")
    print(out_df.to_string(index=False))


if __name__ == "__main__":
    main()

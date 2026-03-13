import time
import math
import os

from scripts.common import (
    get_settings,
    open_beamng,
    setup_utah_ego_lead_pair,
    BeamNGTelemetryLogger,
)

# BASELINE TEMPLATE (copy for most scenarios)
#
# What this baseline does:
#   - Spawns lead + ego at fixed coordinates (from world editor)
#   - Lead starts immediately at a target speed (AI speed set)
#   - Ego starts after a delay, with ACC target speed (Lua tech_ACC)
#   - Optional events (speed step, brake, despawn, etc.)
#   - Logs telemetry to results/raw/<script>_<timestamp>.csv
#   - Auto-stops after MAX_TIME_S seconds
#
# To create new scenarios:
#   - ONLY edit the "SCENARIO EVENTS" section below (and maybe the speeds/timing)
#   - Keep spawning + logger unchanged
# =========================================================

# -----------------------------
# SPAWNS (DO NOT CHANGE without updating from World Editor)
# -----------------------------
EGO_POS = (-883.820, -1162.193, 149.940)
LEAD_POS = (-822.902, -1162.339, 149.939)
YAW_DEG = -90.203

# -----------------------------
# CORE SPEED SETTINGS
# -----------------------------
LEAD_SPEED_KPH_INITIAL = 60.0
LEAD_SPEED_KPH_LATER = 70.0            # used if you enable a "speed step"
EGO_ACC_TARGET_KPH = 80.0              # ego ACC set speed (should be > lead)

# Keep ego AI speed limit slightly above ACC target so AI doesn't clamp ACC.
# This is your "ACC is in charge, AI only steers" trick.
EGO_AI_SPEED_LIMIT_KPH = 90.0

# -----------------------------
# TIMING
# -----------------------------
DT = 0.1
EGO_START_DELAY_S = 3.0                # Good baseline
MAX_TIME_S = 90.0                      # auto-stop after 1.5 min (set to 60 for 1 min)

# -----------------------------
# SCENARIO EVENTS (edit these per scenario)
# -----------------------------
# Set any time to None to disable it.

# Scenario A and C: lead speed step (baseline follow)
LEAD_SPEED_STEP_AT_S = 30.0            # at t=30s, lead goes 60->70. Set None to disable.

# Scenario B: lead hard brake (panic event)
LEAD_BRAKE_AT_S = None                 # e.g. 25.0 to start braking event
LEAD_BRAKE_DURATION_S = 3.0            # how long to brake
LEAD_BRAKE_THROTTLE = 0.0
LEAD_BRAKE_BRAKE = 1.0                 # 1.0 = full brake; try 0.6 for "hard but not lockup"

# Scenario D: lead speeds away faster than ego target (lead not limiting)
LEAD_SPEED_AWAY_AT_S = None            # e.g. 35.0
LEAD_SPEED_AWAY_KPH = 110.0            # lead accelerates away; ego should hold 80

# =========================================================
# Helpers
# =========================================================
def kph_to_mps(kph: float) -> float:
    return kph / 3.6

def speed_from_vel(vel) -> float:
    return math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2)

def get_vehicle_numeric_id(bng, vehicle_name: str) -> int:
    """
    tech_ACC needs a numeric vehicle id.
    BeamNGpy build you have exposes get_current_vehicles_info().
    """
    info = bng.get_current_vehicles_info()

    if isinstance(info, dict):
        if vehicle_name in info and isinstance(info[vehicle_name], dict) and "id" in info[vehicle_name]:
            return int(info[vehicle_name]["id"])
        for _k, v in info.items():
            if isinstance(v, dict) and v.get("name") == vehicle_name and "id" in v:
                return int(v["id"])

    if isinstance(info, list):
        for v in info:
            if isinstance(v, dict) and v.get("name") == vehicle_name and "id" in v:
                return int(v["id"])

    raise RuntimeError(f"Could not find numeric id for vehicle '{vehicle_name}'.")

def acc_start(vehicle, veh_id_int: int, target_kph: float, debug_csv: bool = False):
    target_mps = kph_to_mps(target_kph)
    dbg = "true" if debug_csv else "false"
    vehicle.queue_lua_command(f"extensions.tech_ACC.loadAccWithID({veh_id_int}, {target_mps}, {dbg})")

def acc_change_speed(vehicle, target_kph: float):
    target_mps = kph_to_mps(target_kph)
    vehicle.queue_lua_command(f"extensions.tech_ACC.changeSpeed({target_mps})")

def acc_stop(vehicle):
    vehicle.queue_lua_command("extensions.tech_ACC.unloadACC()")

def main():
    s = get_settings()
    bng = open_beamng(s)

    # --- Spawning/setup: DO NOT TOUCH ---
    run = setup_utah_ego_lead_pair(
        bng,
        scenario_name="baseline_follow",
        ego_pos=EGO_POS,
        lead_pos=LEAD_POS,
        yaw_deg=YAW_DEG,
        settle_s=1.0,
    )
    ego = run.vehicles["ego"]
    lead = run.vehicles["lead"]
    # -----------------------------------

    # Make CSV name match the script file automatically
    script_name = os.path.splitext(os.path.basename(__file__))[0]

    logger = BeamNGTelemetryLogger(
        settings=s,
        script_name=script_name,
        test_description="Baseline follow template: lead starts first; ego starts after delay with ACC; scenario events toggled by *_AT_S.",
        acc_target_kph=EGO_ACC_TARGET_KPH,
        lead_target_kph=LEAD_SPEED_KPH_INITIAL,
    )

    # -----------------------------
    # Lead starts immediately (same as 14)
    # -----------------------------
    lead.ai_drive_in_lane(True)
    lead.ai_set_aggression(0.1)
    lead.ai_set_speed(kph_to_mps(LEAD_SPEED_KPH_INITIAL), mode="set")
    lead.ai_set_mode("span")
    print(f"Lead started: {LEAD_SPEED_KPH_INITIAL:.0f} km/h")

    # -----------------------------
    # Ego should NOT move before EGO_START_DELAY_S
    # Key fix vs your previous 21:
    #   DO NOT set ego ai_set_mode('span') until the start time.
    #   Keep it disabled so it stays stopped, like in 14.
    # -----------------------------
    ego_started = False
    ego_id = get_vehicle_numeric_id(bng, "ego")

    # Pre-configure ego but keep AI disabled so it doesn't roll off at t=0
    ego.ai_drive_in_lane(True)
    ego.ai_set_aggression(0.1)
    ego.ai_set_speed(kph_to_mps(EGO_AI_SPEED_LIMIT_KPH), mode="limit")
    ego.ai_set_mode("disabled")  # <-- THIS is what makes it behave like 14

    # Track event triggers
    lead_sped_up = False
    lead_brake_started = False
    lead_brake_end_t = None
    lead_despawned = False
    lead_sped_away = False

    t0 = time.time()

    try:
        while True:
            time.sleep(DT)
            t = time.time() - t0

            # -----------------------------
            # Start ego at delay (like 14)
            # Order matters:
            #   1) enable AI span so it can steer and move
            #   2) start ACC immediately after
            # This matches your working behavior in 14.
            # -----------------------------
            if (not ego_started) and (t >= EGO_START_DELAY_S):
                ego_started = True

                ego.ai_set_mode("span")  # <-- start driving now (not before)
                acc_start(ego, ego_id, EGO_ACC_TARGET_KPH, debug_csv=False)

                print(f"\nEgo started at t={t:.1f}s: ACC target {EGO_ACC_TARGET_KPH:.0f} km/h")

            # -----------------------------
            # Scenario A and C: lead speed step 60->70 or 60->100
            # -----------------------------
            if (LEAD_SPEED_STEP_AT_S is not None) and (not lead_sped_up) and (t >= LEAD_SPEED_STEP_AT_S):
                lead_sped_up = True
                lead.ai_set_speed(kph_to_mps(LEAD_SPEED_KPH_LATER), mode="set")
                print(f"\nLead speed step: now {LEAD_SPEED_KPH_LATER:.0f} km/h")

            # -----------------------------
            # Scenario B: lead hard brake event
            # -----------------------------
            if (LEAD_BRAKE_AT_S is not None) and (not lead_brake_started) and (t >= LEAD_BRAKE_AT_S):
                lead_brake_started = True
                lead_brake_end_t = t + LEAD_BRAKE_DURATION_S
                print(f"\nLead brake event START (for {LEAD_BRAKE_DURATION_S:.1f}s)")

            if lead_brake_started and (lead_brake_end_t is not None):
                if t <= lead_brake_end_t:
                    lead.control(throttle=LEAD_BRAKE_THROTTLE, brake=LEAD_BRAKE_BRAKE)
                else:
                    lead.control(throttle=0.0, brake=0.0)
                    lead_brake_end_t = None
                    print("\nLead brake event END")

            # -----------------------------
            # Scenario D: lead speeds away fast
            # -----------------------------
            if (LEAD_SPEED_AWAY_AT_S is not None) and (not lead_sped_away) and (t >= LEAD_SPEED_AWAY_AT_S):
                lead_sped_away = True
                lead.ai_set_speed(kph_to_mps(LEAD_SPEED_AWAY_KPH), mode="set")
                print(f"\nLead speeds away: {LEAD_SPEED_AWAY_KPH:.0f} km/h")

            # Log every tick (same format for all scenarios)
            logger.log(t, ego, lead)

            # Auto-stop
            if t >= MAX_TIME_S:
                break

    finally:
        # Stop ACC cleanly
        try:
            acc_stop(ego)
        except Exception:
            pass

        logger.close()
        bng.close()

    print("\nDone.")


if __name__ == "__main__":
    main()
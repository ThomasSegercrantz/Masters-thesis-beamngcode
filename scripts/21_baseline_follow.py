import time
import math
import os

from scripts.common import (
    get_settings,
    open_beamng,
    setup_utah_ego_lead_pair,
    BeamNGTelemetryLogger,
)

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

# Keep ego AI speed limit above ACC target so AI doesn't clamp ACC.
EGO_AI_SPEED_LIMIT_KPH = 120.0

# -----------------------------
# TIMING
# -----------------------------
DT = 0.1
EGO_START_DELAY_S = 5.0
MAX_TIME_S = 90.0

# -----------------------------
# DEBUG PRINTS
# -----------------------------
DEBUG_SPEED_WINDOW = True
DEBUG_LUA_ACC = True
DEBUG_ACTUATION = True
ACTUATION_PRINT_EVERY_S = 1.0


# Scenario A: lead speed step (baseline follow)
LEAD_SPEED_STEP_AT_S = 30.0            # at t=30s, lead goes 60->70. Set None to disable.

# =========================================================
# Helpers
# =========================================================
def kph_to_mps(kph: float) -> float:
    return kph / 3.6


def speed_from_vel(vel) -> float:
    return math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2)


def dist3(a, b) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


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
    vehicle.queue_lua_command(
        f"extensions.tech_ACC.loadAccWithID({veh_id_int}, {target_mps}, {dbg})"
    )


def acc_stop(vehicle):
    vehicle.queue_lua_command("extensions.tech_ACC.unloadACC()")


# =========================================================
# Main
# =========================================================
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

    script_name = os.path.splitext(os.path.basename(__file__))[0]

    logger = BeamNGTelemetryLogger(
        settings=s,
        script_name=script_name,
        test_description=(
            "Baseline follow template: lead starts first; ego starts after delay with ACC; "
            "scenario events toggled by *_AT_S."
        ),
        acc_target_kph=EGO_ACC_TARGET_KPH,
        lead_target_kph=LEAD_SPEED_KPH_INITIAL,
    )

    # -----------------------------
    # Lead starts immediately
    # -----------------------------
    lead.ai_drive_in_lane(True)
    lead.ai_set_aggression(0.3)
    lead.ai_set_speed(kph_to_mps(LEAD_SPEED_KPH_INITIAL), mode="set")
    lead.ai_set_mode("span")
    print(f"Lead started: {LEAD_SPEED_KPH_INITIAL:.0f} km/h")

    # -----------------------------
    # Ego pre-configured but disabled until start time
    # -----------------------------
    ego_started = False
    ego_id = get_vehicle_numeric_id(bng, "ego")

    ego.ai_drive_in_lane(True)
    ego.ai_set_aggression(0.05)
    ego.ai_set_speed(kph_to_mps(EGO_AI_SPEED_LIMIT_KPH), mode="limit")
    ego.ai_set_mode("disabled")

    # Track event triggers
    lead_sped_up = False
    acc_started = False

    t0 = time.time()

    try:
        while True:
            time.sleep(DT)
            t = time.time() - t0

            # Debug: monitor ego/lead speeds around ACC activation window
            if DEBUG_SPEED_WINDOW:
                ego.poll_sensors()
                lead.poll_sensors()

            # -----------------------------
            # Start ego at delay
            # -----------------------------
            if (not ego_started) and (t >= EGO_START_DELAY_S):
                ego_started = True
                ego.ai_set_mode("span")

            # -----------------------------
            # Request ACC only once ego is above threshold
            # -----------------------------
            if ego_started and (not acc_started):
                ego.poll_sensors()
                ego_speed_mps = speed_from_vel(ego.state["vel"])
                ego_speed_kph = ego_speed_mps * 3.6

                if ego_speed_kph >= 78.0:
                    acc_started = True

                    acc_start(ego, ego_id, EGO_ACC_TARGET_KPH, debug_csv=False)

                    print("[ACC_COMMAND_SENT] loadAccWithID called")

            # -----------------------------
            # Debug: follow / actuation diagnostics once ACC is active
            # -----------------------------
            if acc_started:
                ego.poll_sensors()
                lead.poll_sensors()

            # Scenario A: lead speed step 60->70
            if (LEAD_SPEED_STEP_AT_S is not None) and (not lead_sped_up) and (t >= LEAD_SPEED_STEP_AT_S):
                lead_sped_up = True
                lead.ai_set_speed(kph_to_mps(LEAD_SPEED_KPH_LATER), mode="set")
                print(f"\nLead speed step: now {LEAD_SPEED_KPH_LATER:.0f} km/h")


            # Log every tick
            logger.log(t, ego, lead)

            if t >= MAX_TIME_S:
                break

    finally:
        try:
            acc_stop(ego)
        except Exception:
            pass

        logger.close()
        bng.close()

    print("\nDone.")


if __name__ == "__main__":
    main()
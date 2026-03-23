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
LEAD_SPEED_KPH_INITIAL = 70.0
LEAD_SPEED_KPH_FINAL = 45.0
EGO_ACC_TARGET_KPH = 80.0

# -----------------------------
# TIMING
# -----------------------------
DT = 0.1
MAX_TIME_S = 55.0
ACC_TRIGGER_KPH = 78.0

# -----------------------------
# SCENARIO: lead gradual slowdown (not to zero)
# -----------------------------
LEAD_SLOWDOWN_START_AT_S = 30.0
LEAD_SLOWDOWN_DURATION_S = 10.0

# Optional debug
PRINT_STATUS_EVERY_S = 1.0


def kph_to_mps(kph: float) -> float:
    return kph / 3.6


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def speed_from_vel(vel) -> float:
    return math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2)


def get_vehicle_numeric_id(bng, vehicle_name: str) -> int:
    info = bng.get_current_vehicles_info()

    if isinstance(info, dict):
        if vehicle_name in info and isinstance(info[vehicle_name], dict) and "id" in info[vehicle_name]:
            return int(info[vehicle_name]["id"])
        for _, v in info.items():
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


def enable_lead_ai(lead):
    """Apply the same lead AI cruise principle used by the other scenarios."""
    lead.ai_drive_in_lane(True)
    lead.ai_set_aggression(0.1)
    lead.ai_set_speed(kph_to_mps(LEAD_SPEED_KPH_INITIAL), mode="set")
    lead.ai_set_mode("span")


def main():
    s = get_settings()
    bng = open_beamng(s)

    run = setup_utah_ego_lead_pair(
        bng,
        scenario_name="lead_slowdown",
        ego_pos=EGO_POS,
        lead_pos=LEAD_POS,
        yaw_deg=YAW_DEG,
        settle_s=1.0,
    )
    ego = run.vehicles["ego"]
    lead = run.vehicles["lead"]

    script_name = os.path.splitext(os.path.basename(__file__))[0]
    logger = BeamNGTelemetryLogger(
        settings=s,
        script_name=script_name,
        test_description=(
            "Lead slowdown with manual ego. Lead cruises at 60 km/h, ego is fully manual until "
            "78 km/h, then ACC is requested and the user continues steering only. Lead then "
            "smoothly reduces speed from 60 to 30 km/h and remains there."
        ),
        acc_target_kph=EGO_ACC_TARGET_KPH,
        lead_target_kph=LEAD_SPEED_KPH_INITIAL,
    )

    # Lead starts immediately, same principle as 22/24/25.
    enable_lead_ai(lead)
    print(f"Lead started: {LEAD_SPEED_KPH_INITIAL:.0f} km/h")

    # Ego: absolutely no AI before ACC.
    try:
        ego.ai_set_mode("disabled")
    except Exception:
        pass

    ego_id = get_vehicle_numeric_id(bng, "ego")

    acc_started = False
    lead_slowdown_started = False
    lead_slowdown_done = False
    lead_slowdown_t0 = None
    last_status_print_t = -1e9

    t0 = time.time()

    try:
        while True:
            time.sleep(DT)
            t = time.time() - t0

            ego.poll_sensors()
            lead.poll_sensors()

            ego_speed_kph = speed_from_vel(ego.state["vel"]) * 3.6
            lead_speed_kph = speed_from_vel(lead.state["vel"]) * 3.6

            # Start ACC only once ego has manually reached trigger speed.
            if (not acc_started) and (ego_speed_kph >= ACC_TRIGGER_KPH):
                acc_started = True
                acc_start(ego, ego_id, EGO_ACC_TARGET_KPH, debug_csv=False)
                print(
                    f"ACC started at t={t:.1f}s | "
                    f"ego_speed={ego_speed_kph:.1f} km/h | "
                    f"target={EGO_ACC_TARGET_KPH:.0f} km/h"
                )
                print("Now release throttle/brake and steer manually only.")

            # Start the gradual slowdown event.
            if (not lead_slowdown_started) and (t >= LEAD_SLOWDOWN_START_AT_S):
                lead_slowdown_started = True
                lead_slowdown_t0 = t
                print(
                    f"\nLead slowdown START at t={t:.1f}s "
                    f"({LEAD_SPEED_KPH_INITIAL:.0f}->{LEAD_SPEED_KPH_FINAL:.0f} km/h over "
                    f"{LEAD_SLOWDOWN_DURATION_S:.1f}s)"
                )

            # Smoothly reduce lead target speed from initial to final over the slowdown window.
            if lead_slowdown_started and (not lead_slowdown_done):
                elapsed = t - lead_slowdown_t0
                progress = clamp(elapsed / LEAD_SLOWDOWN_DURATION_S, 0.0, 1.0)
                target_kph = LEAD_SPEED_KPH_INITIAL + (
                    LEAD_SPEED_KPH_FINAL - LEAD_SPEED_KPH_INITIAL
                ) * progress

                lead.ai_set_speed(kph_to_mps(target_kph), mode="set")

                if progress >= 1.0:
                    lead.ai_set_speed(kph_to_mps(LEAD_SPEED_KPH_FINAL), mode="set")
                    lead_slowdown_done = True
                    print(
                        f"\nLead slowdown END at t={t:.1f}s | "
                        f"holding {LEAD_SPEED_KPH_FINAL:.0f} km/h"
                    )

            logger.log(t, ego, lead)

            if t - last_status_print_t >= PRINT_STATUS_EVERY_S:
                last_status_print_t = t
                print(
                    f"t={t:5.1f}s | ego={ego_speed_kph:5.1f} km/h | "
                    f"lead={lead_speed_kph:5.1f} km/h | "
                    f"acc={'Y' if acc_started else 'N'} | "
                    f"slowdown={'Y' if lead_slowdown_started and not lead_slowdown_done else 'N'} | "
                    f"slowdown_done={'Y' if lead_slowdown_done else 'N'}"
                )

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

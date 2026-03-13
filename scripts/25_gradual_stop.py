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
EGO_ACC_TARGET_KPH = 80.0

# -----------------------------
# TIMING
# -----------------------------
DT = 0.1
MAX_TIME_S = 55.0
ACC_TRIGGER_KPH = 78.0

# -----------------------------
# SCENARIO: lead gradual stop to zero
# -----------------------------
LEAD_STOP_START_AT_S = 30.0
LEAD_STOP_DURATION_S = 12.0
LEAD_STOP_EPS_KPH = 0.8

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
    """Apply the standard lead AI cruise used by the other scenarios."""
    lead.ai_drive_in_lane(True)
    lead.ai_set_aggression(0.1)
    lead.ai_set_speed(kph_to_mps(LEAD_SPEED_KPH_INITIAL), mode="set")
    lead.ai_set_mode("span")


def hold_lead_stopped(lead):
    """Hold the lead at standstill once it has reached zero."""
    try:
        lead.control(throttle=0.0, brake=0.0, parkingbrake=1.0)
    except TypeError:
        lead.control(throttle=0.0, brake=0.0)


def main():
    s = get_settings()
    bng = open_beamng(s)

    run = setup_utah_ego_lead_pair(
        bng,
        scenario_name="gradual_stop",
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
            "Gradual stop with manual ego. Lead uses AI at 60 km/h, ego is fully manual until "
            "78 km/h, then ACC is requested and user continues steering only. Lead then smoothly "
            "reduces speed to zero and remains stopped."
        ),
        acc_target_kph=EGO_ACC_TARGET_KPH,
        lead_target_kph=LEAD_SPEED_KPH_INITIAL,
    )

    # Lead starts immediately, same principle as 22/24.
    enable_lead_ai(lead)
    print(f"Lead started: {LEAD_SPEED_KPH_INITIAL:.0f} km/h")

    # Ego: absolutely no AI before ACC.
    try:
        ego.ai_set_mode("disabled")
    except Exception:
        pass

    ego_id = get_vehicle_numeric_id(bng, "ego")

    acc_started = False
    lead_stop_started = False
    lead_stopped = False
    lead_stop_completed = False
    lead_ai_disabled_for_hold = False
    lead_stop_t0 = None
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

            # Start the gradual-stop event.
            if (not lead_stop_started) and (t >= LEAD_STOP_START_AT_S):
                lead_stop_started = True
                lead_stop_t0 = t
                print(
                    f"\nLead gradual stop START at t={t:.1f}s "
                    f"(duration {LEAD_STOP_DURATION_S:.1f}s)"
                )

            # Smoothly reduce lead target speed from 60 to 0 over the stop window.
            if lead_stop_started and (not lead_stopped):
                elapsed = t - lead_stop_t0
                progress = clamp(elapsed / LEAD_STOP_DURATION_S, 0.0, 1.0)
                target_kph = LEAD_SPEED_KPH_INITIAL * (1.0 - progress)

                # Let the lane-following AI handle the smooth deceleration profile.
                lead.ai_set_speed(kph_to_mps(target_kph), mode="set")

                # Once the ramp is complete and the car is effectively stopped, hold it still.
                if (progress >= 1.0) and (lead_speed_kph <= LEAD_STOP_EPS_KPH):
                    try:
                        lead.ai_set_mode("disabled")
                        lead_ai_disabled_for_hold = True
                    except Exception:
                        lead_ai_disabled_for_hold = False

                    hold_lead_stopped(lead)
                    lead_stopped = True
                    lead_stop_completed = True
                    print(f"\nLead reached standstill at t={t:.1f}s and will remain stopped")

            elif lead_stopped:
                if not lead_ai_disabled_for_hold:
                    try:
                        lead.ai_set_mode("disabled")
                        lead_ai_disabled_for_hold = True
                    except Exception:
                        pass
                hold_lead_stopped(lead)

            logger.log(t, ego, lead)

            if t - last_status_print_t >= PRINT_STATUS_EVERY_S:
                last_status_print_t = t
                print(
                    f"t={t:5.1f}s | ego={ego_speed_kph:5.1f} km/h | "
                    f"lead={lead_speed_kph:5.1f} km/h | "
                    f"acc={'Y' if acc_started else 'N'} | "
                    f"gradual_stop={'Y' if lead_stop_started and not lead_stopped else 'N'} | "
                    f"stop_done={'Y' if lead_stop_completed else 'N'}"
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

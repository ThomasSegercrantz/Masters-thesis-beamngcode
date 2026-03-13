import json
import os
import time
from beamngpy import Scenario, Vehicle
from scripts.common import get_settings, open_beamng

LEVEL = "utah"
OUTFILE = os.path.join("configs", "spawns.json")

def get_pose_via_lua(bng, vehicle_id: int):
    lua = f"""
    local veh = be:getObjectByID({vehicle_id})
    if veh == nil then return nil end
    local pos = veh:getPosition()
    local rot = quat(veh:getRotation())
    return {{
      pos = {{pos.x, pos.y, pos.z}},
      rot = {{rot.x, rot.y, rot.z, rot.w}}
    }}
    """
    return bng.control.lua(lua)

def main():
    s = get_settings()
    bng = open_beamng(s)

    scenario = Scenario(LEVEL, "acc_spawn_calibration")
    ego = Vehicle("ego", model="etk800")
    scenario.add_vehicle(ego, pos=(0, 0, 0), rot_quat=(0, 0, 0, 1))

    scenario.make(bng)
    bng.scenario.load(scenario)
    bng.scenario.start()

    # give BeamNG a moment to fully spawn the vehicle
    time.sleep(0.5)

    print("\n--- Calibration ---")
    print("1) In BeamNG UI: teleport/move the car onto your chosen straight road segment (Utah)")
    print("2) Align it straight in-lane")
    print("3) Come back here and press ENTER to capture pose\n")
    input("Press ENTER to capture ego pose...")

    res = get_pose_via_lua(bng, ego.vid)
    if res is None:
        raise RuntimeError("Could not get vehicle pose via Lua. Vehicle ID not found.")

    pos = res["pos"]
    rot = res["rot"]

    os.makedirs("configs", exist_ok=True)
    with open(OUTFILE, "w") as f:
        json.dump({"ego_pose": {"pos": pos, "rot_quat": rot}}, f, indent=2)

    print(f"\n✅ Saved {OUTFILE}")
    bng.close()

if __name__ == "__main__":
    main()
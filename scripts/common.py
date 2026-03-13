import os
import math
import time
from dataclasses import dataclass
from dotenv import load_dotenv
from beamngpy import BeamNGpy, Scenario, Vehicle
import csv
from datetime import datetime
from pathlib import Path

load_dotenv()


@dataclass(frozen=True)
class Settings:
    home: str
    host: str
    port: int
    mode: str
    results_dir: str


def get_settings() -> Settings:
    home = os.getenv("BEAMNG_HOME", "").strip()
    if not home:
        raise RuntimeError(
            "BEAMNG_HOME not set. Copy .env.example -> .env and set BEAMNG_HOME to the BeamNG.tech folder."
        )
    return Settings(
        home=home,
        host=os.getenv("BEAMNG_HOST", "localhost").strip(),
        port=int(os.getenv("BEAMNG_PORT", "64256")),
        mode=os.getenv("BEAMNG_MODE", "ui").strip().lower(),
        results_dir=os.getenv("RESULTS_DIR", "results").strip(),
    )


def open_beamng(s: Settings) -> BeamNGpy:
    bng = BeamNGpy(s.host, s.port, home=s.home)
    mode = s.mode

    if mode == "ui":
        bng.open(launch=True)
    elif mode == "headless":
        bng.open(launch=True, headless=True)
    elif mode == "nogfx":
        bng.open(launch=True, nogfx=True)  # fastest, no camera sensors
    else:
        raise ValueError(f"Unknown BEAMNG_MODE: {mode} (use ui/headless/nogfx)")

    return bng


# ----------------------------
# New reusable scenario helpers
# ----------------------------

def rot_quat_from_yaw_deg(yaw_deg: float) -> tuple[float, float, float, float]:
    """
    Convert a yaw angle in degrees (Euler Z) into a quaternion (x,y,z,w).
    This matches the fix you applied in your spawn script.
    """
    yaw = math.radians(yaw_deg)
    return (0.0, -0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@dataclass(frozen=True)
class VehicleSpawn:
    vid: str
    model: str
    pos: tuple[float, float, float]
    yaw_deg: float
    cling: bool = True
    part_config: str | None = None


@dataclass(frozen=True)
class ScenarioRun:
    bng: BeamNGpy
    scenario: Scenario
    vehicles: dict[str, Vehicle]


def _make_vehicle_from_spawn(spawn: VehicleSpawn) -> Vehicle:
    """
    Construct a BeamNGpy Vehicle from our wrapper config.

    BeamNGpy's Vehicle constructor accepts `part_config` for a .pc file path.
    Using a fallback keeps this helper resilient across BeamNGpy variants.
    """
    if spawn.part_config:
        try:
            return Vehicle(spawn.vid, model=spawn.model, part_config=spawn.part_config)
        except TypeError:
            # Fallback for older/different BeamNGpy builds that may ignore part_config
            return Vehicle(spawn.vid, model=spawn.model)
    return Vehicle(spawn.vid, model=spawn.model)


def setup_scenario_with_vehicles(
    bng: BeamNGpy,
    level: str,
    scenario_name: str,
    spawns: list[VehicleSpawn],
    *,
    start: bool = True,
    settle_s: float = 1.0,
) -> ScenarioRun:
    """
    Creates a Scenario, adds vehicles from spawn specs, makes/loads/starts scenario,
    then polls sensors once.

    Returns ScenarioRun with a {vid: Vehicle} map.
    """
    scenario = Scenario(level, scenario_name)

    vehicles: dict[str, Vehicle] = {}
    for s in spawns:
        v = _make_vehicle_from_spawn(s)
        vehicles[s.vid] = v
        scenario.add_vehicle(
            v,
            pos=s.pos,
            rot_quat=rot_quat_from_yaw_deg(s.yaw_deg),
            cling=s.cling,
        )

    scenario.make(bng)
    bng.scenario.load(scenario)

    if start:
        bng.scenario.start()
        if settle_s > 0:
            time.sleep(settle_s)

    # Poll once so callers can immediately read v.state
    for v in vehicles.values():
        try:
            v.poll_sensors()
        except Exception:
            # Some setups don't have sensors configured; state still exists after spawn
            pass

    return ScenarioRun(bng=bng, scenario=scenario, vehicles=vehicles)


def setup_utah_ego_lead_pair(
    bng: BeamNGpy,
    scenario_name: str,
    *,
    ego_pos: tuple[float, float, float],
    lead_pos: tuple[float, float, float],
    yaw_deg: float,
    ego_model: str = "etkc",
    ego_part_config: str | None = "vehicles/etkc/adas_experiment.pc",
    lead_model: str = "etk800",
    lead_part_config: str | None = None,
    settle_s: float = 1.0,
) -> ScenarioRun:
    """
    Convenience wrapper for the exact pattern you’re using in tests:
    Utah + ego + lead, same yaw, cling enabled.

    Defaults now use the ADAS-capable ETK K-series experiment config for ego.
    """
    spawns = [
        VehicleSpawn("ego", ego_model, ego_pos, yaw_deg, True, ego_part_config),
        VehicleSpawn("lead", lead_model, lead_pos, yaw_deg, True, lead_part_config),
    ]
    return setup_scenario_with_vehicles(
        bng,
        level="utah",
        scenario_name=scenario_name,
        spawns=spawns,
        start=True,
        settle_s=settle_s,
    )

def setup_gridmap_ego_lead_pair(
    bng: BeamNGpy,
    scenario_name: str,
    *,
    ego_pos: tuple[float, float, float],
    lead_pos: tuple[float, float, float],
    yaw_deg: float,
    ego_model: str = "etkc",
    ego_part_config: str | None = "vehicles/etkc/adas_experiment.pc",
    lead_model: str = "etk800",
    lead_part_config: str | None = None,
    settle_s: float = 1.0,
) -> ScenarioRun:
    """
    Same helper as Utah version but using Gridmap.

    Useful for straight-line ADAS testing where curvature causes drift.
    """

    spawns = [
        VehicleSpawn("ego", ego_model, ego_pos, yaw_deg, True, ego_part_config),
        VehicleSpawn("lead", lead_model, lead_pos, yaw_deg, True, lead_part_config),
    ]

    return setup_scenario_with_vehicles(
        bng,
        level="smallgrid",
        scenario_name=scenario_name,
        spawns=spawns,
        start=True,
        settle_s=settle_s,
    )


class BeamNGTelemetryLogger:
    """
    Generic CSV telemetry logger for BeamNG experiments.

    Logs:
      - ego & lead speeds
      - accelerations
      - gap distance
      - relative speed
      - throttle / brake / steering
      - positions
    """

    def __init__(
        self,
        settings: Settings,
        script_name: str,
        test_description: str,
        acc_target_kph: float | None = None,
        lead_target_kph: float | None = None,
    ):
        self.settings = settings
        self.script_name = script_name
        self.test_description = test_description
        self.acc_target_kph = acc_target_kph
        self.lead_target_kph = lead_target_kph

        now = datetime.now()
        timestamp = now.strftime("%Y%m%d_%H%M%S")

        raw_dir = Path(settings.results_dir) / "raw"
        raw_dir.mkdir(parents=True, exist_ok=True)

        filename = f"{script_name}_{timestamp}.csv"
        self.path = raw_dir / filename

        self.file = open(self.path, "w", newline="")
        self.writer = csv.writer(self.file)

        # --- Header block (commented metadata) ---
        self.file.write(f"# Script: {script_name}\n")
        self.file.write(f"# Date: {now.isoformat()}\n")
        self.file.write(f"# Description: {test_description}\n")
        if acc_target_kph is not None:
            self.file.write(f"# ACC target (kph): {acc_target_kph}\n")
        if lead_target_kph is not None:
            self.file.write(f"# Lead target (kph): {lead_target_kph}\n")
        self.file.write("# --------------------------------------------\n")

        # --- CSV Header ---
        self.writer.writerow([
            "time_s",
            "ego_speed_mps",
            "ego_accel_mps2",
            "lead_speed_mps",
            "lead_accel_mps2",
            "gap_m",
            "relative_speed_mps",
            "ego_throttle",
            "ego_brake",
            "ego_steering",
            "ego_pos_x",
            "ego_pos_y",
            "ego_pos_z",
            "lead_pos_x",
            "lead_pos_y",
            "lead_pos_z",
        ])

        self.prev_ego_speed = None
        self.prev_lead_speed = None
        self.prev_time = None

    @staticmethod
    def _speed_from_vel(vel):
        return math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)

    @staticmethod
    def _distance(a, b):
        return math.sqrt(
            (a[0]-b[0])**2 +
            (a[1]-b[1])**2 +
            (a[2]-b[2])**2
        )

    def log(self, t, ego, lead):
        ego.poll_sensors()
        lead.poll_sensors()

        ego_vel = ego.state.get("vel")
        lead_vel = lead.state.get("vel")
        ego_pos = ego.state.get("pos")
        lead_pos = lead.state.get("pos")

        if not ego_vel or not lead_vel or not ego_pos or not lead_pos:
            return

        ego_speed = self._speed_from_vel(ego_vel)
        lead_speed = self._speed_from_vel(lead_vel)

        if self.prev_time is None:
            ego_acc = 0.0
            lead_acc = 0.0
        else:
            dt = max(1e-6, t - self.prev_time)
            ego_acc = (ego_speed - self.prev_ego_speed) / dt
            lead_acc = (lead_speed - self.prev_lead_speed) / dt

        gap = self._distance(ego_pos, lead_pos)
        rel_speed = ego_speed - lead_speed

        electrics = ego.state.get("electrics", {})

        throttle = electrics.get("throttle", 0.0)
        brake = electrics.get("brake", 0.0)
        steering = electrics.get("steering", 0.0)

        self.writer.writerow([
            t,
            ego_speed,
            ego_acc,
            lead_speed,
            lead_acc,
            gap,
            rel_speed,
            throttle,
            brake,
            steering,
            ego_pos[0], ego_pos[1], ego_pos[2],
            lead_pos[0], lead_pos[1], lead_pos[2],
        ])

        self.prev_time = t
        self.prev_ego_speed = ego_speed
        self.prev_lead_speed = lead_speed

    def close(self):
        self.file.close()

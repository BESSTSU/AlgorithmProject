"""CoppeliaSim backend over the ZMQ Remote API.

Preview stage: builds the scene from a GridMap, spawns differential-drive robots
and drives a planned scenario with a simple step-by-step sequencer. The ADG
executor replaces `run_plan` in M5/M7.

CoppeliaSim must be running (its ZMQ remote API add-on starts by default):
    python -m robots.coppelia --scenario scenarios/s0.yaml
"""
import argparse
import math
import random
import time
from itertools import combinations, groupby
from pathlib import Path

import yaml

from core.battery import BatteryModel
from core.grid_map import CHARGER, DOOR, FREE, WALL, Cell, GridMap
from core.prioritized import plan_prioritized
from core.scenario import load_scenario
from robots.base import RobotInterface

SCENE_ROOT = "mrpp_scene"
FLOOR_COLOR = [0.85, 0.93, 0.91]
WALL_COLOR = [0.18, 0.31, 0.31]
ROOM_COLOR = [0.93, 0.95, 0.95]
DODGE_COLOR = [0.96, 0.85, 0.63]
CHARGER_COLOR = [0.95, 0.75, 0.10]
WHEEL_COLOR = [0.10, 0.10, 0.10]
ROBOT_COLORS = [[0.95, 0.66, 0.23], [0.12, 0.44, 0.42], [0.75, 0.31, 0.30],
                [0.31, 0.51, 0.74], [0.50, 0.39, 0.64]]
TILE_THICKNESS = 0.002   # cosmetic floor markings, not respondable
CAMERA_DISTANCE = 0.9    # camera offset as a fraction of the larger map side


def cell_to_world(grid: GridMap, cell: Cell, cell_size: float) -> tuple[float, float]:
    """World (x, y) of a cell center. x grows with col; y grows upwards, so row 0 is at the top."""
    r, c = cell
    return (c + 0.5) * cell_size, (grid.height - r - 0.5) * cell_size


def world_to_cell(grid: GridMap, xy: tuple[float, float], cell_size: float) -> Cell:
    x, y = xy
    return grid.height - 1 - math.floor(y / cell_size), math.floor(x / cell_size)


def wheel_speeds(v: float, w: float, wheel_radius: float,
                 wheel_separation: float) -> tuple[float, float]:
    """(left, right) wheel speeds in rad/s for body speed v (m/s) and turn rate w (rad/s)."""
    half = w * wheel_separation / 2
    return (v - half) / wheel_radius, (v + half) / wheel_radius


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class CoppeliaRobot(RobotInterface):
    """Differential-drive robot that turns towards a cell, then drives straight to its center."""

    def __init__(self, world: "CoppeliaWorld", name: str, body: int, left: int, right: int):
        self.name = name
        self.xy = None  # last measured position
        self._world = world
        self._sim = world.sim
        self._body, self._left, self._right = body, left, right
        self._target = None  # world (x, y) of the cell being driven to
        self._turning = False

    def _pose(self) -> tuple[float, float, float]:
        m = self._sim.getObjectMatrix(self._body)
        return m[3], m[7], math.atan2(m[4], m[0])

    def get_cell(self) -> Cell:
        x, y, _ = self._pose()
        return world_to_cell(self._world.grid, (x, y), self._world.cell_size)

    def go_to_cell(self, cell: Cell) -> None:
        self._target = cell_to_world(self._world.grid, cell, self._world.cell_size)
        self._turning = True

    def is_arrived(self) -> bool:
        return self._target is None

    def stop(self) -> None:
        self._target = None
        self._set_wheels(0.0, 0.0)

    def update(self) -> None:
        """One control tick. Call once per simulation step."""
        x, y, yaw = self._pose()
        self.xy = (x, y)
        if self._target is None:
            return
        control, limits = self._world.cfg["control"], self._world.robot_cfg
        dx, dy = self._target[0] - x, self._target[1] - y
        dist = math.hypot(dx, dy)
        if dist < control["pos_tol"]:
            self.stop()
            return
        error = math.atan2(dy, dx) - yaw
        error = math.atan2(math.sin(error), math.cos(error))  # wrap to [-pi, pi]
        if self._turning and abs(error) < control["ang_tol"]:
            self._turning = False
        v = 0.0 if self._turning else min(limits["cruise_lin_vel"], control["kp_lin"] * dist)
        w = _clamp(control["kp_ang"] * error, limits["cruise_ang_vel"])
        model = self._world.cfg["model"]
        self._set_wheels(*wheel_speeds(v, w, model["wheel_radius"], model["wheel_separation"]))

    def _set_wheels(self, left: float, right: float) -> None:
        limit = self._world.robot_cfg["max_lin_vel"] / self._world.cfg["model"]["wheel_radius"]
        self._sim.setJointTargetVelocity(self._left, _clamp(left, limit))
        self._sim.setJointTargetVelocity(self._right, _clamp(right, limit))


class CoppeliaWorld:
    def __init__(self, grid: GridMap, config: dict):
        # Imported here so the pure helpers above work without the client installed.
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        self.grid = grid
        self.cell_size = config["cell_size"]
        self.robot_cfg = config["robot"]
        self.cfg = config["coppelia"]
        self.sim = RemoteAPIClient(self.cfg["host"], self.cfg["port"]).require("sim")
        self.robots: dict[str, CoppeliaRobot] = {}

    # --- scene -----------------------------------------------------------------

    def build_scene(self) -> None:
        """Replace the scene content with a floor, walls and cell markings for the map."""
        sim, grid, cs = self.sim, self.grid, self.cell_size
        if sim.getSimulationState() != sim.simulation_stopped:
            sim.stopSimulation()
            while sim.getSimulationState() != sim.simulation_stopped:
                time.sleep(0.1)
        self._remove("/Floor")  # the default 5 x 5 m floor is smaller than the map
        self._remove("/" + SCENE_ROOT)
        root = sim.createDummy(TILE_THICKNESS)
        sim.setObjectAlias(root, SCENE_ROOT)
        sim.setObjectInt32Param(root, sim.objintparam_visibility_layer, 0)

        width, height = grid.width * cs, grid.height * cs
        wall_h, floor_t = self.cfg["wall_height"], self.cfg["floor_thickness"]
        cuboid = sim.primitiveshape_cuboid

        def wall(size_x, size_y, x, y):
            self._shape(cuboid, [size_x, size_y, wall_h], [x, y, wall_h / 2], WALL_COLOR,
                        static=True, respondable=True, parent=root)

        # The floor extends one cell past the map so the border walls stand on it.
        self._shape(cuboid, [width + 2 * cs, height + 2 * cs, floor_t],
                    [width / 2, height / 2, -floor_t / 2], FLOOR_COLOR,
                    static=True, respondable=True, parent=root)
        # Border, as if the map were surrounded by wall cells.
        wall(width + 2 * cs, cs, width / 2, -cs / 2)
        wall(width + 2 * cs, cs, width / 2, height + cs / 2)
        wall(cs, height, -cs / 2, height / 2)
        wall(cs, height, width + cs / 2, height / 2)

        for r, row in enumerate(grid.rows):
            y = (grid.height - r - 0.5) * cs
            # One box per horizontal run of wall cells.
            for is_wall, run in groupby(range(grid.width), key=lambda c: row[c] == WALL):
                cols = list(run)
                if is_wall:
                    wall(len(cols) * cs, cs, (cols[0] + cols[-1] + 1) / 2 * cs, y)
            for c in range(grid.width):
                color = self._marking_color((r, c))
                if color is not None:
                    x, _ = cell_to_world(grid, (r, c), cs)
                    self._shape(cuboid, [cs, cs, TILE_THICKNESS], [x, y, TILE_THICKNESS / 2],
                                color, static=True, respondable=False, parent=root)
        self._aim_camera(width, height)

    def _marking_color(self, cell: Cell):
        symbol = self.grid.symbol(cell)
        if symbol == CHARGER:
            return CHARGER_COLOR
        if symbol == DOOR or self.grid.is_side_pocket(cell):
            return DODGE_COLOR
        if symbol not in (WALL, FREE):
            return ROOM_COLOR
        return None

    def _aim_camera(self, width: float, height: float) -> None:
        """Point the default camera at the map from the south, 45 degrees down."""
        sim = self.sim
        camera = sim.getObject("/DefaultCamera", {"noError": True})
        if camera == -1:
            return
        d = CAMERA_DISTANCE * max(width, height)
        s = math.sqrt(0.5)
        # Columns of the rotation: camera x = world -x, y = up-and-north, z = view direction.
        sim.setObjectMatrix(camera, [-1, 0, 0, width / 2,
                                     0, s, s, height / 2 - d,
                                     0, s, -s, d])

    def _shape(self, kind, size, position, color, static, respondable,
               parent=None, orientation=None) -> int:
        sim = self.sim
        handle = sim.createPrimitiveShape(kind, size)
        sim.setObjectInt32Param(handle, sim.shapeintparam_static, int(static))
        sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, int(respondable))
        sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, color)
        if orientation is not None:
            sim.setObjectOrientation(handle, orientation)
        sim.setObjectPosition(handle, position)
        if parent is not None:
            sim.setObjectParent(handle, parent, True)
        return handle

    def _remove(self, path: str) -> None:
        handle = self.sim.getObject(path, {"noError": True})
        if handle != -1:
            self.sim.removeObjects(self.sim.getObjectsInTree(handle))

    # --- robots ----------------------------------------------------------------

    def add_robot(self, name: str, cell: Cell, color) -> CoppeliaRobot:
        """Spawn a differential-drive robot on `cell`, facing +x."""
        sim, model = self.sim, self.cfg["model"]
        self._remove("/" + name)
        radius = model["wheel_radius"]
        bx, by, bz = model["body_size"]
        body = self._shape(sim.primitiveshape_cuboid, [bx, by, bz],
                           [0, 0, model["body_clearance"] + bz / 2], color,
                           static=False, respondable=True)
        sim.setShapeMass(body, model["body_mass"])
        sim.setObjectAlias(body, name)

        axle = [-math.pi / 2, 0, 0]  # turns the joint and wheel z axes onto the robot's +y
        motors = []
        for side in (1, -1):  # left, right
            position = [0, side * model["wheel_separation"] / 2, radius]
            joint = sim.createJoint(sim.joint_revolute, sim.jointmode_dynamic, 0)
            sim.setObjectInt32Param(joint, sim.objintparam_visibility_layer, 0)
            sim.setObjectOrientation(joint, axle)
            sim.setObjectPosition(joint, position)
            sim.setObjectParent(joint, body, True)
            wheel = self._shape(sim.primitiveshape_cylinder,
                                [2 * radius, 2 * radius, model["wheel_width"]], position,
                                WHEEL_COLOR, static=False, respondable=True,
                                parent=joint, orientation=axle)
            sim.setShapeMass(wheel, model["wheel_mass"])
            sim.setObjectInt32Param(joint, sim.jointintparam_dynctrlmode, sim.jointdynctrl_velocity)
            sim.setJointTargetForce(joint, model["motor_torque"])
            sim.setJointTargetVelocity(joint, 0)
            motors.append(joint)

        caster_radius = model["caster_radius"]
        for side in (1, -1):  # front and rear ball casters on free spherical joints
            position = [side * model["caster_offset"], 0, caster_radius]
            ball = sim.createJoint(sim.joint_spherical, sim.jointmode_dynamic, 0)
            sim.setObjectInt32Param(ball, sim.objintparam_visibility_layer, 0)
            sim.setObjectPosition(ball, position)
            sim.setObjectParent(ball, body, True)
            caster = self._shape(sim.primitiveshape_spheroid, [2 * caster_radius] * 3, position,
                                 WHEEL_COLOR, static=False, respondable=True, parent=ball)
            sim.setShapeMass(caster, model["caster_mass"])

        x, y = cell_to_world(self.grid, cell, self.cell_size)
        sim.setObjectPosition(body, [x, y, sim.getObjectPosition(body)[2]])
        robot = CoppeliaRobot(self, name, body, motors[0], motors[1])
        self.robots[name] = robot
        return robot

    # --- simulation ------------------------------------------------------------

    def start(self) -> None:
        self.sim.setStepping(True)
        self.sim.startSimulation()

    def step(self) -> None:
        """Run every robot's controller once, then advance the simulation one step."""
        for robot in self.robots.values():
            robot.update()
        self.sim.step()

    def pause(self) -> None:
        """Pause instead of stopping, so the final state stays visible."""
        for robot in self.robots.values():
            robot.stop()
        self.sim.pauseSimulation()


def run_plan(world: CoppeliaWorld, paths: dict[str, list[Cell]],
             batteries: dict[str, list[int]], realtime: bool) -> float:
    """Drive the robots through their planned paths; return the closest approach in meters.

    Temporary sequencing for the preview: plan steps run one at a time, and within
    a step a robot only enters a cell once no other robot holds it.
    """
    def at(name, t):
        return paths[name][min(t, len(paths[name]) - 1)]

    dt = world.sim.getSimulationTimeStep()
    next_tick = time.perf_counter()
    closest = math.inf
    makespan = max(len(p) for p in paths.values()) - 1
    for t in range(makespan):
        held = {name: {at(name, t)} for name in paths}  # cells each robot occupies
        pending = {n: at(n, t + 1) for n in paths if at(n, t + 1) != at(n, t)}
        moving = set()
        while pending or moving:
            for name, target in list(pending.items()):
                if all(target not in cells for other, cells in held.items() if other != name):
                    world.robots[name].go_to_cell(target)
                    held[name].add(target)
                    moving.add(name)
                    del pending[name]
            if not moving:
                raise RuntimeError(f"robots block each other at plan step {t}")
            world.step()
            for a, b in combinations(world.robots.values(), 2):
                closest = min(closest, math.dist(a.xy, b.xy))
            for name in list(moving):
                if world.robots[name].is_arrived():
                    held[name] = {at(name, t + 1)}
                    moving.discard(name)
            if realtime:
                next_tick += dt
                time.sleep(max(0.0, next_tick - time.perf_counter()))
        for name, robot in world.robots.items():
            robot.battery = batteries[name][min(t + 1, len(batteries[name]) - 1)]
        status = "   ".join(f"{n} {at(n, t + 1)} bat {r.battery:2d}" for n, r in world.robots.items())
        print(f"t={t + 1:2d}  sim {world.sim.getSimulationTime():6.1f} s   {status}")
    return closest


def main():
    parser = argparse.ArgumentParser(description="Plan a scenario and drive it in CoppeliaSim.")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--scenario", default="scenarios/s0.yaml")
    parser.add_argument("--fast", action="store_true", help="do not pace the run to real time")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    grid = GridMap.from_file(config_path.parent / config["map"])
    battery = BatteryModel(**config["battery"])
    robots = load_scenario(args.scenario)
    planner = config["planner"]
    result = plan_prioritized(grid, robots, battery, planner["t_max"], planner["priority"],
                              planner["max_restarts"], random.Random(config["experiments"]["seed"]))
    if result.paths is None:
        raise SystemExit(f"planning failed: {result.failure} ({result.failed_robot})")
    paths = result.paths
    batteries = {r.name: battery.trace(grid, paths[r.name], r.battery) for r in robots}

    world = CoppeliaWorld(grid, config)
    world.build_scene()
    for i, robot in enumerate(robots):
        world.add_robot(robot.name, robot.start, ROBOT_COLORS[i % len(ROBOT_COLORS)])
    world.start()
    try:
        closest = run_plan(world, paths, batteries, realtime=not args.fast)
    finally:
        world.pause()
    for robot in robots:
        cell, goal = world.robots[robot.name].get_cell(), robot.tasks[-1]
        print(f"{robot.name}: ended in {cell}, goal {goal} -> {'OK' if cell == goal else 'MISSED'}")
    print(f"closest approach between robot centers: {closest:.3f} m")


if __name__ == "__main__":
    main()

"""Pure kinematic TrackingCenter trajectory; no simulation or sensor dependencies.

Time is relative simulation time in seconds. At an interior waypoint the next
segment's velocity applies. At and after the last waypoint velocity is zero.
Position remains continuous; acceleration is undefined at velocity jumps.
"""

from bisect import bisect_right
from copy import deepcopy
from dataclasses import dataclass
import math


SCHEMA = "mro_stage1_waypoints_v1"


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def _xyz(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must have exactly three coordinates")
    return tuple(_number(v, f"{name}[{i}]") for i, v in enumerate(value))


@dataclass(frozen=True)
class Trajectory:
    points: tuple
    speeds: tuple
    velocities: tuple
    starts: tuple
    ends: tuple
    duration_s: float


def validate_trajectory(config):
    """Validate explicit units, frame, finite coordinates, speed and bounds."""
    if not isinstance(config, dict) or config.get("schema") != SCHEMA:
        raise ValueError("Unsupported waypoint schema")
    if config.get("frame_id") != "world" or config.get("reference") != "TrackingCenter":
        raise ValueError("Waypoints must reference TrackingCenter in world")
    if config.get("position_unit") != "m" or config.get("time_unit") != "s":
        raise ValueError("Explicit m and s units required")
    if config.get("interpolation") != "piecewise_constant_speed":
        raise ValueError("Only piecewise_constant_speed interpolation supported")
    if config.get("boundary_rule") != "next_segment_then_terminal_stop":
        raise ValueError("Explicit next-segment boundary rule required")
    orientation = config.get("fixed_orientation", {})
    if orientation.get("frame_id") != "world":
        raise ValueError("Fixed orientation must be in world")
    if _xyz(orientation.get("rpy_degrees"), "rpy_degrees") != (0.0, 0.0, 180.0):
        raise ValueError("This stage keeps level roll/pitch and fixed yaw 180 degrees")
    if orientation.get("orientation_xyzw") != [0.0, 0.0, 1.0, 0.0]:
        raise ValueError("Fixed quaternion must agree with level yaw 180 degrees")
    bounds = config.get("position_bounds_m", {})
    lower = _xyz(bounds.get("min"), "position_bounds_m.min")
    upper = _xyz(bounds.get("max"), "position_bounds_m.max")
    if any(lo >= hi for lo, hi in zip(lower, upper)):
        raise ValueError("Each position lower bound must be less than upper bound")
    items = config.get("waypoints_m")
    if not isinstance(items, list) or not 2 <= len(items) <= 1000:
        raise ValueError("A trajectory requires between 2 and 1000 waypoints")
    points = []
    for index, item in enumerate(items):
        point = _xyz(item, f"waypoint[{index}]")
        if any(not lo <= x <= hi for lo, x, hi in zip(lower, point, upper)):
            raise ValueError(f"Waypoint {index} is outside configured position bounds")
        points.append(point)
    default_speed = _number(config.get("speed_m_s"), "speed_m_s")
    if default_speed <= 0:
        raise ValueError("speed_m_s must be positive")
    explicit_speeds = config.get("segment_speeds_m_s")
    if explicit_speeds is None:
        speeds = [default_speed] * (len(points) - 1)
    else:
        if not isinstance(explicit_speeds, list) or len(explicit_speeds) != len(points) - 1:
            raise ValueError("segment_speeds_m_s must contain one speed per segment")
        speeds = [_number(v, "segment speed") for v in explicit_speeds]
        if any(v <= 0 for v in speeds):
            raise ValueError("Every segment speed must be positive")
    velocities, starts, ends = [], [], []
    elapsed = 0.0
    for index, (a, b, speed) in enumerate(zip(points, points[1:], speeds)):
        delta = tuple(y - x for x, y in zip(a, b))
        distance = math.hypot(*delta)
        if not math.isfinite(distance) or distance <= 1e-9:
            raise ValueError(f"Segment {index} must have finite length greater than 1e-9 m")
        duration = distance / speed
        end = elapsed + duration
        if not math.isfinite(end) or end <= elapsed:
            raise ValueError(f"Segment {index} has unrepresentable duration")
        starts.append(elapsed)
        ends.append(end)
        velocities.append(tuple(x / distance * speed for x in delta))
        elapsed = end
    return Trajectory(tuple(points), tuple(speeds), tuple(velocities),
                      tuple(starts), tuple(ends), elapsed)


def prepare(config):
    """Return a serializable validated route plus its exact arrival schedule."""
    trajectory = validate_trajectory(config)
    result = deepcopy(config)
    result.update({
        "arrival_times_s": [0.0, *trajectory.ends],
        "duration_s": trajectory.duration_s,
        "segment_velocities_m_s": [list(v) for v in trajectory.velocities],
        "segment_speeds_m_s": list(trajectory.speeds),
    })
    return result


def sample(trajectory, t):
    """Return requested scene center state at relative simulation time t.

    This is a planned/applied transform request. An actual scene GT publisher
    must read the scene after applying it rather than labeling this as readback.
    """
    if isinstance(trajectory, dict):
        trajectory = validate_trajectory(trajectory)
    if not isinstance(trajectory, Trajectory):
        raise TypeError("Use validate_trajectory before sampling")
    t = _number(t, "simulation time")
    if t < 0:
        raise ValueError("Relative simulation time must be nonnegative")
    terminal = t >= trajectory.duration_s
    boundary = t == 0.0 or t in trajectory.ends
    if terminal:
        position, velocity, speed, segment = trajectory.points[-1], (0.0, 0.0, 0.0), 0.0, None
    else:
        segment = bisect_right(trajectory.ends, t)
        elapsed = t - trajectory.starts[segment]
        velocity = trajectory.velocities[segment]
        position = tuple(x + v * elapsed for x, v in zip(trajectory.points[segment], velocity))
        speed = trajectory.speeds[segment]
    return {
        "simulation_time_s": t,
        "frame_id": "world",
        "reference": "TrackingCenter",
        "position_m": list(position),
        "orientation_xyzw": [0.0, 0.0, 1.0, 0.0],
        "velocity_m_s": list(velocity),
        "speed_m_s": speed,
        "segment_id": segment,
        "boundary": boundary,
        "terminal_stop": terminal,
        "acceleration_m_s2": None if boundary else [0.0, 0.0, 0.0],
        "acceleration_note": "undefined_at_boundary" if boundary else "constant_velocity_interval",
    }

"""Continuous-path timing ablation on shared, scorer-only geometry.

The geometry and limits form a deterministic planning estimate. The estimates
do not model the tracking, recovery, or control delays of the Nav2 controller.
"""

import math


MODEL_ID = 'continuous_v2'
ROBOT_RADIUS_M = 0.28
MAX_SPEED_M_S = 0.30
LINEAR_ACCEL_M_S2 = 0.60
MAX_TURN_RAD_S = 1.0
ANGULAR_ACCEL_RAD_S2 = 3.2
SIMPLIFICATION_TOLERANCE_M = 0.10
MAX_BLEND_TRIM_M = 0.30
BLEND_INTERVALS = 16
BLEND_QUADRATURE_STEPS = 512
GEOMETRY_EPSILON_M = 1e-9
BLEND_QUADRATURE_CLEARANCE_M = 1e-8
COLLINEAR_ROUNDOFF_ULPS = 8
COLLINEAR_EPSILON_RAD = 1e-9
UTURN_MARGIN_RAD = 1e-3
STOPPED_LINEAR_THRESHOLD_M_S = 1e-3
STRAIGHT_ANGULAR_THRESHOLD_RAD_S = 1e-3
MOVING_YAW_TOLERANCE_RAD = 1e-3
SPEED_ROUNDING_TOLERANCE = 1e-12

TIMING_MODEL = dict(
    id=MODEL_ID,
    scope='scorer_only',
    geometry='clearance_checked_rdp_and_smooth_curvature_blends',
    clearance_basis='known_free_circular_footprint_over_grid_cell_squares_for_changes',
    source_geometry_basis='original_nav2_polyline',
    footprint_radius_m=ROBOT_RADIUS_M,
    simplification_tolerance_m=SIMPLIFICATION_TOLERANCE_M,
    minimum_source_segment_m=GEOMETRY_EPSILON_M,
    collinear_roundoff_ulps=COLLINEAR_ROUNDOFF_ULPS,
    max_blend_trim_m=MAX_BLEND_TRIM_M,
    blend_subintervals=BLEND_INTERVALS,
    blend_quadrature='composite_simpson',
    blend_quadrature_steps=BLEND_QUADRATURE_STEPS,
    blend_quadrature_clearance_m=BLEND_QUADRATURE_CLEARANCE_M,
    uturn_margin_rad=UTURN_MARGIN_RAD,
    negligible_turn_rad=COLLINEAR_EPSILON_RAD,
    clearance_expansion_m=GEOMETRY_EPSILON_M,
    max_speed_m_s=MAX_SPEED_M_S,
    max_turn_rad_s=MAX_TURN_RAD_S,
    linear_accel_m_s2=LINEAR_ACCEL_M_S2,
    angular_accel_rad_s2=ANGULAR_ACCEL_RAD_S2,
    motion_profile='continuous_interval_envelope_with_analytic_interval_times',
    angular_acceleration_budget='equal_halves_for_curvature_and_tangential_terms',
    stopped_linear_threshold_m_s=STOPPED_LINEAR_THRESHOLD_M_S,
    straight_angular_threshold_rad_s=STRAIGHT_ANGULAR_THRESHOLD_RAD_S,
    moving_yaw_tolerance_rad=MOVING_YAW_TOLERANCE_RAD,
    speed_rounding_tolerance=SPEED_ROUNDING_TOLERANCE,
    terminal_state='rest',
)


class InvalidTimingPath(ValueError):
    """Common geometry cannot be computed from the given inputs."""


class InvalidMotionState(ValueError):
    """The initial velocity falls outside the timing model's supported domain."""


def _angle_delta(target, source):
    return math.remainder(target - source, math.tau)


def validate_motion_state(linear_speed, angular_speed):
    """Validate body-frame longitudinal speed and signed yaw rate."""
    try:
        linear_speed, angular_speed = float(linear_speed), float(angular_speed)
    except (TypeError, OverflowError, ValueError) as exc:
        raise InvalidMotionState('initial velocities must be finite numbers') from exc
    if not all(map(math.isfinite, (linear_speed, angular_speed))):
        raise InvalidMotionState('initial velocities must be finite numbers')
    if linear_speed < -STOPPED_LINEAR_THRESHOLD_M_S:
        raise InvalidMotionState('reverse longitudinal speed is unsupported')
    if (linear_speed > MAX_SPEED_M_S + SPEED_ROUNDING_TOLERANCE
            or abs(angular_speed) > MAX_TURN_RAD_S + SPEED_ROUNDING_TOLERANCE):
        raise InvalidMotionState('initial velocity exceeds a modeled speed limit')
    if abs(linear_speed) <= STOPPED_LINEAR_THRESHOLD_M_S:
        return 0., math.copysign(min(abs(angular_speed), MAX_TURN_RAD_S), angular_speed)
    if abs(angular_speed) > STRAIGHT_ANGULAR_THRESHOLD_RAD_S:
        raise InvalidMotionState('simultaneous forward motion and yaw rate are unsupported')
    return min(linear_speed, MAX_SPEED_M_S), 0.


def _path_points(points):
    normalized = []
    try:
        for x, y in points:
            point = float(x), float(y)
            if not all(map(math.isfinite, point)):
                raise InvalidTimingPath('path coordinates must be finite')
            if not normalized or point != normalized[-1]:
                normalized.append(point)
    except (TypeError, OverflowError, ValueError) as exc:
        if isinstance(exc, InvalidTimingPath):
            raise
        raise InvalidTimingPath('expected pairs of finite path coordinates') from exc
    if not normalized:
        raise InvalidTimingPath('a path requires at least one position')
    for a, b in zip(normalized, normalized[1:]):
        length = math.dist(a, b)
        if not math.isfinite(length) or length < GEOMETRY_EPSILON_M:
            raise InvalidTimingPath('path has a segment outside the supported length range')
    return normalized


def _point_square_distance_sq(point, xmin, ymin, xmax, ymax):
    x, y = point
    return max(xmin - x, 0., x - xmax)**2 + max(ymin - y, 0., y - ymax)**2


def _segment_intersects_square(a, b, xmin, ymin, xmax, ymax):
    lower, upper = 0., 1.
    for start, end, minimum, maximum in zip(a, b, (xmin, ymin), (xmax, ymax)):
        change = end - start
        if change == 0.:
            if not minimum <= start <= maximum:
                return False
            continue
        first, second = sorted(((minimum - start) / change, (maximum - start) / change))
        lower, upper = max(lower, first), min(upper, second)
        if lower > upper:
            return False
    return True


def _chord_coordinates(point, start, unit):
    x, y = point[0] - start[0], point[1] - start[1]
    return x * unit[0] + y * unit[1], x * unit[1] - y * unit[0]


def _segment_square_distance_sq(a, b, unit, length, xmin, ymin, xmax, ymax):
    distance_sq = min(_point_square_distance_sq(p, xmin, ymin, xmax, ymax) for p in (a, b))
    if distance_sq == 0. or length == 0.:
        return distance_sq
    if _segment_intersects_square(a, b, xmin, ymin, xmax, ymax):
        return 0.
    for corner in ((xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)):
        along, across = _chord_coordinates(corner, a, unit)
        distance_sq = min(distance_sq, across**2 + max(-along, 0., along - length)**2)
    return distance_sq


class _Clearance:
    """Test a swept disk in the map's own axes, including cell corners."""

    def __init__(self, grid):
        self.grid = grid
        try:
            valid = (math.isfinite(grid.resolution) and grid.resolution > 0.
                     and grid.width > 0 and grid.height > 0
                     and len(grid.data) == grid.width * grid.height
                     and all(map(math.isfinite, (grid.origin_x, grid.origin_y, grid.origin_yaw))))
        except (AttributeError, TypeError, OverflowError) as exc:
            raise InvalidTimingPath('invalid occupancy grid') from exc
        if not valid:
            raise InvalidTimingPath('invalid occupancy grid')
        self.cos = math.cos(grid.origin_yaw)
        self.sin = math.sin(grid.origin_yaw)

    def local(self, point):
        dx, dy = point[0] - self.grid.origin_x, point[1] - self.grid.origin_y
        return self.cos * dx + self.sin * dy, -self.sin * dx + self.cos * dy

    def corridor(self, start, end, radius):
        a, b = self.local(start), self.local(end)
        if not all(map(math.isfinite, (*a, *b))):
            return False
        if a[0] > b[0]:
            a, b = b, a
        grid, radius = self.grid, radius + GEOMETRY_EPSILON_M
        resolution = grid.resolution
        if (a[0] <= radius or b[0] >= grid.width * resolution - radius
                or min(a[1], b[1]) <= radius
                or max(a[1], b[1]) >= grid.height * resolution - radius):
            return False

        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        unit = (dx / length, dy / length) if length else (0., 0.)
        radius_sq = radius**2
        for col in range(math.floor((a[0] - radius) / resolution),
                         math.floor((b[0] + radius) / resolution) + 1):
            xmin, xmax = col * resolution, (col + 1) * resolution
            if dx:
                start_t = max(0., (xmin - radius - a[0]) / dx)
                end_t = min(1., (xmax + radius - a[0]) / dx)
                y1, y2 = a[1] + start_t * dy, a[1] + end_t * dy
            else:
                y1, y2 = a[1], b[1]
            for row in range(math.floor((min(y1, y2) - radius) / resolution),
                             math.floor((max(y1, y2) + radius) / resolution) + 1):
                value = grid.data[row * grid.width + col]
                if 0 <= value < 50:
                    continue
                if _segment_square_distance_sq(
                        a, b, unit, length, xmin, row * resolution,
                        xmax, (row + 1) * resolution) <= radius_sq:
                    return False
        return True



def connect_start(points, start_xy, grid, max_deviation_m=0.20):
    """Keep subcell planner starts as explicit proxies; certify larger connectors."""
    normalized = _path_points(points)
    try:
        start = tuple(float(value) for value in start_xy)
    except (TypeError, ValueError, OverflowError) as exc:
        raise InvalidTimingPath('invalid observed start position') from exc
    if len(start) != 2 or not all(map(math.isfinite, start)):
        raise InvalidTimingPath('invalid observed start position')
    if not math.isfinite(max_deviation_m) or max_deviation_m <= 0:
        raise InvalidTimingPath('invalid start connector bound')
    clearance = _Clearance(grid)
    original = normalized[0]
    deviation = math.dist(start, original)
    proxy_bound = min(max_deviation_m, grid.resolution / math.sqrt(2.))
    if deviation > max_deviation_m:
        raise InvalidTimingPath('planner start exceeds connector bound')
    if deviation == 0.:
        basis = 'identical_start'
    elif deviation < GEOMETRY_EPSILON_M:
        normalized[0] = start
        basis = 'sub_nanometer_roundoff'
    elif deviation <= proxy_bound:
        # No synthetic traversal or footprint certificate is claimed here.
        # The caller permits this bounded position approximation only at rest.
        basis = 'bounded_planner_start_proxy'
    elif clearance.corridor(start, original, ROBOT_RADIUS_M):
        normalized.insert(0, start)
        basis = 'certified_known_free_footprint_corridor'
    else:
        raise InvalidTimingPath('start connector lacks known-free footprint clearance')
    return [list(point) for point in normalized], dict(
        observed_start_xy=list(start), planner_start_xy=list(original),
        deviation_m=deviation, max_deviation_m=max_deviation_m,
        proxy_bound_m=proxy_bound, proxy_requires_stopped_translation=True,
        clearance_basis=basis, footprint_radius_m=ROBOT_RADIUS_M,
    )


def _smoothstep(t):
    return t * t * (3. - 2. * t)


def _blend_length(trim, turn):
    def cosine(t):
        return math.cos(turn * (_smoothstep(t) - 0.5))

    steps = BLEND_QUADRATURE_STEPS
    integral = math.fsum(
        (1 if i in (0, steps) else 4 if i % 2 else 2) * cosine(i / steps)
        for i in range(steps + 1)
    ) / (3. * steps)
    return 2. * trim * math.cos(turn / 2.) / integral


def _shift(point, distance, heading):
    return [point[0] + distance * math.cos(heading), point[1] + distance * math.sin(heading)]


def _pivot(point, entry, exit, turn, reason):
    return dict(kind='pivot', at_xy=list(point), entry_heading_rad=entry,
                exit_heading_rad=exit, signed_turn_rad=turn, reason=reason)


def _curvature_scale(primitive):
    return 6. * abs(primitive['signed_turn_rad']) / primitive['length_m']


def _turn_limited_time(primitive):
    travel_time = primitive['length_m'] / MAX_SPEED_M_S
    if primitive['kind'] == 'straight':
        return travel_time
    peak_curvature = _curvature_scale(primitive) / 4.
    if peak_curvature * MAX_SPEED_M_S <= MAX_TURN_RAD_S:
        return travel_time

    # Solve v_max * |kappa(t)| = omega_max. Translation limits the outer
    # intervals. Integrate curvature analytically on the central interval.
    first_crossing = (1. - math.sqrt(
        1. - MAX_TURN_RAD_S / (peak_curvature * MAX_SPEED_M_S))) / 2.
    central_turn = abs(primitive['signed_turn_rad']) * (1. - 2. * _smoothstep(first_crossing))
    return max(travel_time, 2. * first_crossing * travel_time + central_turn / MAX_TURN_RAD_S)


def _translation_intervals(primitive, index):
    length = primitive['length_m']
    if primitive['kind'] == 'straight':
        yield dict(primitive_index=index, t0=0., t1=1., ds_m=length,
                   curvature_bound_rad_m=0., curvature_derivative_bound_rad_m2=0.,
                   speed_cap_m_s=MAX_SPEED_M_S, accel_cap_m_s2=LINEAR_ACCEL_M_S2)
        return
    scale = _curvature_scale(primitive)
    for interval in range(BLEND_INTERVALS):
        t0, t1 = interval / BLEND_INTERVALS, (interval + 1) / BLEND_INTERVALS
        peak = min(t1, max(t0, 0.5))
        curvature = scale * peak * (1. - peak)
        derivative = scale * max(abs(1. - 2. * t0), abs(1. - 2. * t1)) / length
        # omega = kappa * v, alpha = kappa' * v^2 + kappa * a.
        # Bound the magnitude of each acceleration term on the entire interval.
        speed = min(MAX_SPEED_M_S, MAX_TURN_RAD_S / curvature,
                    math.sqrt(ANGULAR_ACCEL_RAD_S2 / (2. * derivative)))
        acceleration = min(LINEAR_ACCEL_M_S2, ANGULAR_ACCEL_RAD_S2 / (2. * curvature))
        yield dict(primitive_index=index, t0=t0, t1=t1, ds_m=length / BLEND_INTERVALS,
                   curvature_bound_rad_m=curvature, curvature_derivative_bound_rad_m2=derivative,
                   speed_cap_m_s=speed, accel_cap_m_s2=acceleration)


def _interval_profile(distance, start, end, cap, acceleration):
    """Fastest symmetric-acceleration profile with fixed endpoint speeds."""
    peak = min(cap, math.sqrt(acceleration * distance + (start**2 + end**2) / 2.))
    # A feasible envelope can differ from its recomputed peak by a final ulp.
    peak = max(peak, start, end)
    accelerating = (peak - start) / acceleration
    decelerating = (peak - end) / acceleration
    ramp_distance = ((peak + start) * accelerating + (peak + end) * decelerating) / 2.
    cruising = max(0., distance - ramp_distance) / peak if peak else 0.
    return peak, accelerating, cruising, decelerating


def _solve_translation(intervals, initial_speed):
    if not intervals:
        return
    caps = [interval['speed_cap_m_s'] for interval in intervals]
    speeds = [initial_speed, *(min(a, b) for a, b in zip(caps, caps[1:])), 0.]
    for index, interval in enumerate(intervals):
        speeds[index + 1] = min(speeds[index + 1], math.sqrt(
            speeds[index]**2 + 2. * interval['accel_cap_m_s2'] * interval['ds_m']))
    for index in reversed(range(len(intervals))):
        interval = intervals[index]
        speeds[index] = min(speeds[index], caps[index], math.sqrt(
            speeds[index + 1]**2 + 2. * interval['accel_cap_m_s2'] * interval['ds_m']))
    if initial_speed - speeds[0] > SPEED_ROUNDING_TOLERANCE:
        raise InvalidMotionState('initial speed cannot reach a downstream limit or stop in time')
    speeds[0] = initial_speed
    for index, interval in enumerate(intervals):
        start, end = speeds[index:index + 2]
        peak, accelerating, cruising, decelerating = _interval_profile(
            interval['ds_m'], start, end, caps[index], interval['accel_cap_m_s2'])
        interval.update(entry_speed_m_s=start, exit_speed_m_s=end, peak_speed_m_s=peak,
                        accelerating_s=accelerating, cruising_s=cruising,
                        decelerating_s=decelerating,
                        dt_s=math.fsum((accelerating, cruising, decelerating)))


def _solve_pivot(primitive, index, initial_rate):
    turn = primitive['signed_turn_rad']
    direction = math.copysign(1., turn)
    remaining, rate = abs(turn), initial_rate * direction
    acceleration = ANGULAR_ACCEL_RAD_S2
    brake_time = brake_turn = 0.
    if rate < 0. or rate**2 / (2. * acceleration) > remaining:
        brake_time = abs(rate) / acceleration
        brake_turn = initial_rate * brake_time / 2.
        remaining = turn - brake_turn
        direction = math.copysign(1., remaining)
        remaining, rate = abs(remaining), 0.
    peak, accelerating, cruising, decelerating = _interval_profile(
        remaining, rate, 0., MAX_TURN_RAD_S, acceleration)
    return dict(primitive_index=index, entry_rate_rad_s=initial_rate,
                exit_rate_rad_s=0., peak_rate_after_brake_rad_s=direction * peak,
                braking_s=brake_time, braking_turn_rad=brake_turn,
                remaining_turn_rad=direction * remaining, accelerating_s=accelerating,
                cruising_s=cruising, decelerating_s=decelerating,
                dt_s=math.fsum((brake_time, accelerating, cruising, decelerating)))


def _yaw(value):
    try:
        return math.remainder(float(value), math.tau)
    except (TypeError, OverflowError, ValueError) as exc:
        raise InvalidTimingPath('start and goal yaws must be finite numbers') from exc

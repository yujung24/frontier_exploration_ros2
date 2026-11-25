"""Continuous timing with geometric-deviation-first path simplification.

The frozen v2 kernel supplies footprint clearance and numerical timing
primitives. This module assembles those primitives from v3 source edges.
"""

import math

from . import continuous_timing as v2
from .continuous_timing import (
    BLEND_QUADRATURE_CLEARANCE_M, COLLINEAR_EPSILON_RAD, COLLINEAR_ROUNDOFF_ULPS,
    GEOMETRY_EPSILON_M, MAX_BLEND_TRIM_M, MAX_SPEED_M_S, MAX_TURN_RAD_S,
    MOVING_YAW_TOLERANCE_RAD, ROBOT_RADIUS_M, SIMPLIFICATION_TOLERANCE_M,
    UTURN_MARGIN_RAD, InvalidMotionState, InvalidTimingPath, _angle_delta,
    _blend_length, _chord_coordinates, _Clearance, _path_points, _pivot,
    _shift, _solve_pivot, _solve_translation, _translation_intervals,
    _turn_limited_time, _yaw, connect_start, validate_motion_state,
)
from .endpoint_geometry import terminal_roundtrip

MODEL_ID = 'continuous_v3'
TIMING_MODEL = dict(
    v2.TIMING_MODEL,
    id=MODEL_ID,
    timing_kernel=v2.MODEL_ID,
    simplification_split_order='max_finite_chord_deviation_then_nonmonotone_progress',
    endpoint_cleanup='optional_certified_float32_grid_roundtrip_of_exact_goal',
)


def _simplified_edges(points, clearance):
    """Split large bends before testing monotone progress along local chords."""
    edges, splits = [], []
    pending = [(0, len(points) - 1)]
    while pending:
        start, end = pending.pop()
        if start == end:
            continue
        if end == start + 1:
            edges.append((start, end, 'original_nav2'))
            continue
        a, b = points[start], points[end]
        length = math.dist(a, b)
        if not math.isfinite(length):
            raise InvalidTimingPath('nonfinite chord length')
        split = (start + end) // 2
        if length < GEOMETRY_EPSILON_M:
            reason = 'degenerate_chord'
        else:
            coordinate_scale = max(length, *map(abs, (*a, *b)))
            roundoff = min(COLLINEAR_ROUNDOFF_ULPS * math.ulp(coordinate_scale),
                           GEOMETRY_EPSILON_M / COLLINEAR_ROUNDOFF_ULPS)
            unit = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
            previous, max_deviation = 0., 0.
            first_nonmonotone = None
            for index in range(start + 1, end):
                along, across = _chord_coordinates(points[index], a, unit)
                if first_nonmonotone is None and not previous <= along <= length + roundoff:
                    first_nonmonotone = index
                previous = along
                deviation = math.hypot(across, max(-along, 0., along - length))
                if deviation > max_deviation:
                    split, max_deviation = index, deviation
            if max_deviation > SIMPLIFICATION_TOLERANCE_M:
                reason = 'geometric_deviation'
            elif first_nonmonotone is not None:
                split, reason = first_nonmonotone, 'nonmonotone_progress'
            elif max_deviation <= roundoff:
                edges.append((start, end, 'original_nav2'))
                continue
            elif clearance.corridor(a, b, ROBOT_RADIUS_M):
                edges.append((start, end, 'certified_shortcut'))
                continue
            else:
                reason = 'shortcut_clearance'
        splits.append(dict(source_start=start, source_end=end, split_index=split, reason=reason))
        pending.extend(((split, end), (start, split)))
    return edges, splits


def _geometry_from_edges(points, edges, start_yaw, goal_yaw, clearance, linear_speed, angular_speed):
    """Assemble the v2 primitives from the selected source edges."""
    if not edges:
        if linear_speed:
            raise InvalidMotionState('a moving start requires a path with translation')
        turn = _angle_delta(goal_yaw, start_yaw)
        return [_pivot(points[0], start_yaw, goal_yaw, turn, 'goal')] if turn or angular_speed else []

    headings = [math.atan2(points[end][1] - points[start][1], points[end][0] - points[start][0])
                for start, end, _ in edges]
    lengths = [math.dist(points[start], points[end]) for start, end, _ in edges]
    trims = [0.] * (len(edges) + 1)
    bends = [None] * (len(edges) + 1)
    initial_turn = _angle_delta(headings[0], start_yaw)
    if linear_speed and abs(initial_turn) > MOVING_YAW_TOLERANCE_RAD:
        raise InvalidMotionState('moving start yaw is not aligned with the first path heading')
    if not linear_speed and (angular_speed or abs(initial_turn) > COLLINEAR_EPSILON_RAD):
        bends[0] = _pivot(points[0], start_yaw, headings[0], initial_turn, 'initial_heading')
    for index in range(1, len(edges)):
        vertex = points[edges[index][0]]
        entry, exit_heading = headings[index - 1], headings[index]
        turn = _angle_delta(exit_heading, entry)
        if abs(turn) <= COLLINEAR_EPSILON_RAD:
            continue
        trim = min(MAX_BLEND_TRIM_M, lengths[index - 1] / 2., lengths[index] / 2.)
        if abs(turn) >= math.pi - UTURN_MARGIN_RAD:
            bends[index] = _pivot(vertex, entry, exit_heading, turn, 'u_turn')
        elif not clearance.corridor(vertex, vertex,
                                    trim + ROBOT_RADIUS_M + BLEND_QUADRATURE_CLEARANCE_M):
            bends[index] = _pivot(vertex, entry, exit_heading, turn, 'blend_clearance')
        else:
            # In the bend bisector frame, |heading offset| <= |turn|/2.
            # Each half moves monotonically in both coordinates, from radius
            # trim toward the midpoint (or back). Its swept disk is therefore
            # contained in the tested ball around the original vertex, with
            # extra clearance for numerical error in the evaluated arc length.
            trims[index] = trim
            bends[index] = dict(
                kind='blend', center_xy=list(vertex), trim_m=trim,
                start_xy=_shift(vertex, -trim, entry), end_xy=_shift(vertex, trim, exit_heading),
                entry_heading_rad=entry, exit_heading_rad=exit_heading,
                signed_turn_rad=turn, length_m=_blend_length(trim, turn),
                clearance_basis='certified_blend_ball',
            )
    terminal_turn = _angle_delta(goal_yaw, headings[-1])
    if abs(terminal_turn) > COLLINEAR_EPSILON_RAD:
        bends[-1] = _pivot(points[-1], headings[-1], goal_yaw, terminal_turn, 'goal')

    geometry = []
    if bends[0]:
        geometry.append(bends[0])
    for index, (start, end, basis) in enumerate(edges):
        length = lengths[index] - trims[index] - trims[index + 1]
        if length > 0.:
            heading = headings[index]
            geometry.append(dict(
                kind='straight', source_start=start, source_end=end, clearance_basis=basis,
                start_xy=_shift(points[start], trims[index], heading),
                end_xy=_shift(points[end], -trims[index + 1], heading),
                entry_heading_rad=heading, exit_heading_rad=heading, length_m=length,
            ))
        if bends[index + 1]:
            geometry.append(bends[index + 1])
    return geometry


def _edges_with_endpoint(points, clearance, simplify, goal_xy, endpoint_mode):
    cleanup = terminal_roundtrip(points, goal_xy, clearance, endpoint_mode)
    if cleanup['applied']:
        indices = [*range(len(points) - 2), len(points) - 1]
        edges, splits = simplify([points[index] for index in indices], clearance)
        edges = [(indices[start], indices[end], basis) for start, end, basis in edges]
        start, end, basis = edges[-1]
        # A collinear merge can extend across an earlier source span that
        # was never certified. Only relabel it after checking that full span.
        if (basis == 'original_nav2' and start < cleanup['replacement_source_start']
                and not clearance.corridor(points[start], points[end], ROBOT_RADIUS_M)):
            cleanup.update(applied=False, reason='merged_edge_lacks_footprint_clearance')
        else:
            if basis == 'original_nav2':
                edges[-1] = (start, end, 'certified_terminal_roundtrip')
            for split in splits:
                for key in ('source_start', 'source_end', 'split_index'):
                    split[key] = indices[split[key]]
            cleanup['certified_edge_source_start'] = start
            return edges, splits, cleanup
    edges, splits = simplify(points, clearance)
    return edges, splits, cleanup


def _evaluate_with_edges(points, start_yaw, goal_yaw, grid, linear_speed, angular_speed, simplify,
                         *, goal_xy=None, endpoint_mode='grid_roundtrip'):
    points = _path_points(points)
    start_yaw, goal_yaw = _yaw(start_yaw), _yaw(goal_yaw)
    if not all(map(math.isfinite, (start_yaw, goal_yaw))):
        raise InvalidTimingPath('start and goal yaws must be finite numbers')
    speed, rate = validate_motion_state(linear_speed, angular_speed)
    clearance = _Clearance(grid)
    edges, splits, cleanup = _edges_with_endpoint(points, clearance, simplify, goal_xy, endpoint_mode)
    geometry = _geometry_from_edges(points, edges, start_yaw, goal_yaw, clearance, speed, rate)
    translation, pivots, block = [], [], []
    block_initial_speed = speed
    station = 0.
    for index, primitive in enumerate(geometry):
        if primitive['kind'] == 'pivot':
            _solve_translation(block, block_initial_speed)
            block.clear()
            block_initial_speed = 0.
            pivots.append(_solve_pivot(primitive, index, rate if index == 0 else 0.))
            primitive.update(station_m=station, distance_only_s=0.,
                             turn_limited_s=abs(primitive['signed_turn_rad']) / MAX_TURN_RAD_S)
            continue
        intervals = list(_translation_intervals(primitive, index))
        length = primitive['length_m']
        for interval in intervals:
            interval.update(s0_m=station + length * interval['t0'],
                            s1_m=station + length * interval['t1'])
        station += length
        translation.extend(intervals)
        block.extend(intervals)
        primitive.update(distance_only_s=length / MAX_SPEED_M_S,
                         turn_limited_s=_turn_limited_time(primitive))
    _solve_translation(block, block_initial_speed)

    length = math.fsum(primitive.get('length_m', 0.) for primitive in geometry)
    distance_time = math.fsum(primitive['distance_only_s'] for primitive in geometry)
    turn_time = math.fsum(primitive['turn_limited_s'] for primitive in geometry)
    translation_time = math.fsum(interval['dt_s'] for interval in translation)
    pivot_time = math.fsum(pivot['dt_s'] for pivot in pivots)
    accel_time = math.fsum((translation_time, pivot_time))
    if not all(map(math.isfinite, (distance_time, turn_time, accel_time))):
        raise InvalidTimingPath('path timing exceeds the supported numeric range')
    return dict(
        time_distance_v3_s=distance_time,
        time_turn_v3_s=turn_time,
        time_accel_v3_s=accel_time,
        timing_witness_v3=dict(
            model_id=MODEL_ID, source_points_xy=[list(point) for point in points],
            start_yaw_rad=start_yaw, goal_yaw_rad=goal_yaw,
            measured_linear_speed_m_s=float(linear_speed),
            measured_angular_speed_rad_s=float(angular_speed),
            initial_speed_m_s=speed, initial_rate_rad_s=rate, common_length_m=length,
            geometry=geometry, translation_intervals=translation, pivots=pivots,
            simplified_edges=[dict(source_start=start, source_end=end, clearance_basis=basis)
                              for start, end, basis in edges],
            simplification_splits=splits,
            endpoint_cleanup=cleanup,
            accel_translation_s=translation_time, accel_rotation_s=pivot_time,
        ),
    )


def evaluate_path(points, start_yaw, goal_yaw, grid, linear_speed=0., angular_speed=0.,
                  *, goal_xy=None, endpoint_mode='grid_roundtrip'):
    """Return the three timing estimates and their shared source-indexed witness."""
    return _evaluate_with_edges(points, start_yaw, goal_yaw, grid, linear_speed, angular_speed,
                                _simplified_edges, goal_xy=goal_xy, endpoint_mode=endpoint_mode)

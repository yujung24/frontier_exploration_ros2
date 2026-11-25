"""Attest one terminal waypoint consistent with grid resolution roundoff."""

import math
import struct

from .continuous_timing import InvalidTimingPath, ROBOT_RADIUS_M


MAX_TERMINAL_HOP_M = 1e-6
MAX_LOCAL_ROUNDOFF_M = 1e-9
LOCAL_ROUNDOFF_ULPS = 16


def _float32(value):
    return struct.unpack('=f', struct.pack('=f', value))[0]


def _float32_half_spacing(value):
    bits = struct.unpack('=I', struct.pack('=f', value))[0]
    if not 0 < bits < 0x7f7fffff or _float32(value) != value:
        return None
    below = struct.unpack('=f', struct.pack('=I', bits - 1))[0]
    above = struct.unpack('=f', struct.pack('=I', bits + 1))[0]
    return max(value - below, above - value) / 2.


def terminal_roundtrip(points, goal_xy, clearance, mode):
    """Return evidence for keeping or removing exactly the penultimate point."""
    if mode not in ('disabled', 'grid_roundtrip'):
        raise ValueError(f'Unsupported endpoint mode: {mode!r}')
    evidence = dict(mode=mode, applied=False, requested_goal_xy=None)

    def keep(reason):
        return dict(evidence, reason=reason)

    if goal_xy is not None:
        try:
            x, y = goal_xy
            goal = float(x), float(y)
        except (TypeError, ValueError, OverflowError) as exc:
            raise InvalidTimingPath('requested goal must contain two finite coordinates') from exc
        if not all(map(math.isfinite, goal)):
            raise InvalidTimingPath('requested goal must contain two finite coordinates')
        evidence['requested_goal_xy'] = list(goal)
    if mode == 'disabled':
        return keep('disabled')
    if goal_xy is None:
        return keep('missing_requested_goal')
    if points[-1] != goal:
        return keep('requested_goal_mismatch')
    if len(points) < 3:
        return keep('insufficient_source_points')
    previous, penultimate, terminal = points[-3:]
    distance = math.dist(penultimate, terminal)
    if not 0. < distance <= MAX_TERMINAL_HOP_M:
        return keep('terminal_hop_outside_bound')
    if min(math.dist(previous, penultimate), math.dist(previous, terminal)) <= MAX_TERMINAL_HOP_M:
        return keep('multiple_short_terminal_segments')

    grid = clearance.grid
    resolution = grid.resolution
    try:
        half_spacing = _float32_half_spacing(resolution)
    except (OverflowError, struct.error):
        half_spacing = None
    if half_spacing is None:
        return keep('resolution_not_supported_float32')
    local_sample = clearance.local(penultimate)
    local_goal = clearance.local(terminal)
    if not all(map(math.isfinite, (*local_sample, *local_goal))):
        return keep('insufficient_coordinate_precision')
    scale = max(map(abs, (*penultimate, *terminal, grid.origin_x, grid.origin_y,
                          *local_sample, *local_goal, resolution)))
    roundoff = LOCAL_ROUNDOFF_ULPS * math.ulp(scale)
    if not math.isfinite(roundoff) or roundoff > MAX_LOCAL_ROUNDOFF_M:
        return keep('insufficient_coordinate_precision')
    axis_steps = tuple(round(value / resolution) for value in local_sample)
    if not any(axis_steps) or any(abs(value - step * resolution) > roundoff
                                  for value, step in zip(local_sample, axis_steps)):
        return keep('penultimate_off_grid_lattice')
    squared_steps = sum(step**2 for step in axis_steps)
    inferred_resolution = math.fsum(value * step for value, step in zip(local_goal, axis_steps)) / squared_steps
    if (inferred_resolution <= 0. or _float32(inferred_resolution) != resolution
            or any(abs(value - step * inferred_resolution) > roundoff
                   for value, step in zip(local_goal, axis_steps))):
        return keep('goal_inconsistent_with_float32_roundtrip')
    displacement_bound = math.sqrt(squared_steps) * half_spacing + 4. * roundoff
    if distance > displacement_bound:
        return keep('terminal_hop_exceeds_roundtrip_bound')
    evidence.update(
        terminal_hop_m=distance, float32_roundtrip_bound_m=displacement_bound,
        coordinate_roundoff_m=roundoff, lattice_steps=list(axis_steps),
        grid_resolution_m=resolution, inferred_resolution_m=inferred_resolution,
        source_penultimate_xy=list(penultimate),
    )
    if not clearance.corridor(previous, terminal, ROBOT_RADIUS_M):
        return keep('replacement_lacks_footprint_clearance')
    return dict(evidence, applied=True, reason='certified_float32_grid_roundtrip',
                removed_source_index=len(points) - 2,
                replacement_source_start=len(points) - 3, replacement_source_end=len(points) - 1,
                clearance_basis='certified_terminal_roundtrip', footprint_radius_m=ROBOT_RADIUS_M)

"""V10 target-lane groups adapter for multimode strategy_engine.py.

Importantly, preserve the V2 select_lanes(mask)[side]['kind'] contract.
Internal geometry: lane_decision.decode -> first join broken components,
then pick the closest-to-90 anchor on each side, then absorb spatially
aligned near-angle target segments (not arbitrarily parallel lanes).
If geometrically linked target line has >1 disconnected fragments => dash;
otherwise dash pixels / all target pixels >3% => dash.
"""
from __future__ import annotations
from lane_decision import decode


def select_lanes(mask, ratio_threshold=.03, *,
                 secondary_angle_tolerance=3.,
                 secondary_lateral_tolerance=42.,
                 secondary_max_gap_frac=.45,
                 parallel_overlap_lateral_tolerance=16.,
                 **geometry_kwargs):
    """Return V2-compatible left/right structures plus V10 diagnostics."""
    decoded = decode(mask, dash_threshold=ratio_threshold,
                     secondary_angle_tolerance=secondary_angle_tolerance,
                     secondary_lateral_tolerance=secondary_lateral_tolerance,
                     secondary_max_gap_frac=secondary_max_gap_frac,
                     parallel_overlap_lateral_tolerance=parallel_overlap_lateral_tolerance,
                     **geometry_kwargs)
    result = {}
    for side in ('left', 'right'):
        line = decoded['selected_lines'][side]
        result[side] = {
            'kind': decoded[side],
            'angle': decoded[f'{side}_angle'],
            'dash_ratio': decoded[f'{side}_dash_ratio'],
            'fragments': decoded[f'{side}_fragments'],
            'xref': decoded[f'{side}_x'],
            'dash_pixels': int(line['dash_pixels']) if line else 0,
            'solid_pixels': int(line['solid_pixels']) if line else 0,
            'target_count': decoded[f'{side}_target_count'],
            'target_line_count': decoded[f'{side}_fragments'],
            'decision_basis': line.get('decision_basis') if line else None,
            'anchor_angle': line['anchor_angle_deg'] if line else None,
            'target_members': line['target_members'] if line else [],
        }
    result['candidate_count'] = decoded['candidate_count']
    result['detected_lines'] = decoded['detected_lines']
    result['decision_rule'] = decoded['decision_rule']
    return result

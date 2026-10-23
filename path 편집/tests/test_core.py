"""Numerical invariants; no display server or matplotlib is required."""
from dataclasses import replace

import numpy as np
import pytest

from pac_path_editor.core import (
    Config, Correction, GateConfig, PathData, apply_corrections, arc_lengths,
    boundary_weights, build_drag_candidate, clip_norm, evaluate_path, generate_paths,
    interpolate_at_xi, local_weights, max_corresponding_displacement, min_jerk,
    raw_field, record_gain, spatial_gate,
)


def drag(config, originals, records, delta, xi=.5, mode="outbound", group="g"):
    selected = "return" if mode == "return" else "outbound"
    return build_drag_candidate(originals, records, config, selected, xi, np.array(delta), mode, group)


def offset(config, originals, records, xi=.5, path="outbound"):
    return evaluate_path(originals[path], records, config, xi) - interpolate_at_xi(originals[path], xi)


def test_round_trip_endpoints_arrays_and_min_jerk():
    config = Config()
    paths = generate_paths(config)
    out, ret = paths.values()
    np.testing.assert_array_equal(out.points[[0, -1]], [config.base, config.target])
    np.testing.assert_array_equal(ret.points[[0, -1]], [config.target, config.base])
    np.testing.assert_allclose(out.points[::-1], ret.points, atol=2e-15)
    assert not np.shares_memory(out.points, ret.points)
    assert not np.shares_memory(out.xi, ret.xi)
    assert not np.shares_memory(out.times, ret.times)
    np.testing.assert_allclose(out.xi, min_jerk(out.times / out.times[-1]), atol=1e-15)
    np.testing.assert_allclose(ret.xi, 1 - out.xi, atol=1e-15)
    assert np.all(np.diff(out.xi) > 0) and np.all(np.diff(ret.xi) < 0)
    ret.points[2, 0] = 9
    assert out.points[-3, 0] != 9


def test_bump_xi_is_original_arclength_not_time_progress():
    config = Config(bump_height=.16)
    paths = generate_paths(config, "bump")
    out, ret = paths.values()
    np.testing.assert_allclose(out.xi, arc_lengths(out.points) / out.length)
    assert not np.allclose(out.xi, min_jerk(out.times / out.times[-1]))
    np.testing.assert_allclose(out.points[[0, -1]], ret.points[[-1, 0]])
    assert np.ptp(ret.points[:, 2]) == 0
    assert np.max(out.points[:, 2]) == pytest.approx(config.base[2] + .16)


def test_cosine_support_and_smooth_boundary():
    np.testing.assert_allclose(local_weights(np.array([0, .1, .2, .3]), .2), [1, .5, 0, 0])
    q = np.linspace(0, 1, 101)
    b = boundary_weights(q, .6, .1)
    assert b[0] == b[-1] == 0
    assert b[50] == 1
    assert np.all((b >= 0) & (b <= 1))
    eps = 1e-6
    assert float(boundary_weights(eps, .6, .1)) / eps < .001


def test_near_endpoints_request_attenuates_smoothly_and_endpoints_stay_fixed():
    config = Config()
    originals = generate_paths(config)
    xi = .035
    records = drag(config, originals, [], [0, 0, .1], xi=xi, mode="both")
    paths = apply_corrections(originals, records, config)
    for key in paths:
        np.testing.assert_array_equal(paths[key].points[[0, -1]], originals[key].points[[0, -1]])
        expected = float(boundary_weights(xi, originals[key].length, config.endpoint_protection)) * .1
        assert offset(config, originals, records, xi=xi, path=key)[2] == pytest.approx(expected)
        assert 0 < expected < .1


@pytest.mark.parametrize("mode", ["outbound", "both"])
def test_exact_endpoint_core_call_never_changes_memory(mode):
    config = Config()
    originals = generate_paths(config)
    prior = drag(config, originals, [], [0, 0, .08], xi=.2, mode=mode)
    before = [record.to_dict() for record in prior]
    result = drag(config, originals, prior, [0, 0, .001], xi=0., mode=mode, group="endpoint")
    assert [record.to_dict() for record in result] == before


def test_norm_input_preserves_direction_and_overlaps_obey_total_limit():
    config = Config(drag_limit=.12, total_offset_limit=.19)
    originals = generate_paths(config)
    requested = np.array([.2, -.3, .4])
    records = drag(config, originals, [], requested)
    np.testing.assert_allclose(offset(config, originals, records), requested / np.linalg.norm(requested) * .12)
    for i in range(6):
        records = drag(config, originals, records, requested, xi=.45 + .01 * i, group=f"g{i}")
    paths = apply_corrections(originals, records, config)
    for key, path in paths.items():
        delta = path.points - interpolate_at_xi(originals[key], path.xi)
        assert np.linalg.norm(delta, axis=1).max() <= .19 + 1e-14
        assert np.isfinite(path.points).all()


def test_eight_then_three_centimetres_is_eleven():
    config = Config()
    originals = generate_paths(config)
    first = drag(config, originals, [], [0, 0, .08])
    second = drag(config, originals, first, [0, 0, .03], group="second")
    assert len(second) == 1
    np.testing.assert_allclose(offset(config, originals, second), [0, 0, .11], atol=1e-14)
    np.testing.assert_allclose(second[0].center, [0.3, 0, .20], atol=1e-14)


def test_retained_different_mode_contribution_is_not_saved_twice():
    config = Config()
    originals = generate_paths(config)
    both = drag(config, originals, [], [0, 0, .08], mode="both", group="both")
    records = drag(config, originals, both, [0, 0, .03], group="one")
    assert len(records) == 3
    np.testing.assert_allclose(offset(config, originals, records), [0, 0, .11], atol=1e-14)
    np.testing.assert_allclose(offset(config, originals, records, path="return"), [0, 0, .08], atol=1e-14)
    newest = next(r for r in records if r.group_id == "one")
    np.testing.assert_allclose(newest.delta, [0, 0, .03], atol=1e-14)
    repeated = drag(config, originals, records, [0, 0, .02], group="two")
    assert len(repeated) == 3
    assert {r.group_id for r in repeated} == {"both", "two"}
    np.testing.assert_allclose(offset(config, originals, repeated), [0, 0, .13], atol=1e-14)
    np.testing.assert_allclose(next(r for r in repeated if r.group_id == "two").delta, [0, 0, .05], atol=1e-14)


def test_different_paths_never_absorb_each_other():
    config = Config()
    originals = generate_paths(config)
    first = drag(config, originals, [], [0, 0, .02], mode="return", group="return_only")
    records = drag(config, originals, first, [0, 0, .04], mode="outbound", group="out_only")
    assert len(records) == 2
    assert first[0].to_dict() == records[0].to_dict()
    np.testing.assert_allclose(offset(config, originals, records, path="return"), [0, 0, .02], atol=1e-14)


def test_single_path_edit_cannot_mutate_other_path():
    config = Config()
    originals = generate_paths(config)
    records = drag(config, originals, [], [0, .04, .08])
    paths = apply_corrections(originals, records, config)
    np.testing.assert_array_equal(paths["return"].points, originals["return"].points)
    assert not np.shares_memory(paths["return"].points, originals["return"].points)


@pytest.mark.parametrize("mode", ["isotropic", "anisotropic"])
def test_gate_core_falloff_outside_continuity_and_segment_minimum(mode):
    gate = GateConfig(mode=mode, radius=.2, radius_xy=.2, radius_z=.1, alpha=.5)
    radius_z = .2 if mode == "isotropic" else .1
    points = np.array([[-1., 0, 0], [1., 0, 0]])
    assert spatial_gate(points, np.array([0, 0, 0]), gate) == 1
    assert spatial_gate(points, np.array([0, 0, radius_z * .5]), gate) == 1
    assert spatial_gate(points, np.array([0, 0, radius_z * .75]), gate) == pytest.approx(.5)
    assert spatial_gate(points, np.array([0, 0, radius_z]), gate) == 0
    assert spatial_gate(points, np.array([0, 0, radius_z * 2]), gate) == 0
    for boundary in (.5, 1.):
        a = spatial_gate(points, np.array([0, 0, radius_z * (boundary - 1e-6)]), gate)
        b = spatial_gate(points, np.array([0, 0, radius_z * (boundary + 1e-6)]), gate)
        assert abs(a - b) < 1e-9
    dense = np.column_stack([np.linspace(-1, 1, 501), np.zeros((501, 2))])
    assert spatial_gate(dense, np.array([0, 0, radius_z * .75]), gate) == pytest.approx(.5)


def test_source_local_gate_preserves_drag_far_from_original_and_reuse_is_gated():
    config = Config(gate=GateConfig(mode="isotropic", radius=.04), total_offset_limit=.3)
    originals = generate_paths(config)
    first = drag(config, originals, [], [0, 0, .08])
    second = drag(config, originals, first, [0, 0, .03], group="second")
    latest = next(r for r in second if r.group_id == "second")
    assert spatial_gate(originals["outbound"].points, latest.center, config.gate) == 0
    assert record_gain(latest, originals["outbound"], config) == 1
    np.testing.assert_allclose(offset(config, originals, second), [0, 0, .11], atol=1e-14)
    shifted = originals["outbound"].copy()
    shifted.points[:, 2] += .11
    assert record_gain(latest, shifted, config) == pytest.approx(.5)
    farther = shifted.copy()
    farther.points[:, 2] += .2
    assert record_gain(latest, farther, config) == 0


def test_gate_attenuated_keep_uses_raw_contribution_once():
    config = Config(gate=GateConfig(mode="isotropic", radius=.2, alpha=.5))
    originals = generate_paths(config)
    old = drag(config, originals, [], [0, 0, .08], mode="both", group="old")
    for record in old:
        record.source_fingerprint = "a-different-original"
        record.center[2] += .15
    assert record_gain(old[0], originals["outbound"], config) == pytest.approx(.5)
    assert offset(config, originals, old)[2] == pytest.approx(.04)
    new = drag(config, originals, old, [0, 0, .03], group="new")
    assert offset(config, originals, new)[2] == pytest.approx(.07)
    assert next(r for r in new if r.group_id == "new").delta[2] == pytest.approx(.03)


def test_saturated_retained_raw_field_requires_unclipped_storage_residual():
    config = Config(total_offset_limit=.1, drag_limit=.1)
    originals = generate_paths(config)
    retained = drag(config, originals, [], [0, 0, .08], mode="both", group="keep")
    for record in retained:
        record.delta *= 5  # Valid reusable raw field, heavily saturated on this source.
    new = drag(config, originals, retained, [0, 0, -.03], group="pullback")
    newest = next(r for r in new if r.group_id == "pullback")
    assert newest.delta[2] == pytest.approx(-.33)
    assert np.linalg.norm(newest.delta) > config.drag_limit
    assert offset(config, originals, new)[2] == pytest.approx(.07)
    paths = apply_corrections(originals, new, config)
    for key, path in paths.items():
        assert np.max(np.linalg.norm(path.points - interpolate_at_xi(originals[key], path.xi), axis=1)) <= .1 + 1e-14


def test_both_noncoincident_xi_preserves_each_existing_visible_center():
    config = Config(sample_dt=.07, bump_height=.16)
    originals = generate_paths(config, "bump")
    prior = drag(config, originals, [], [0, .04, .06], xi=.45, mode="both", group="old")
    before = apply_corrections(originals, prior, config)
    xi = float(before["outbound"].xi[len(before["outbound"].xi) // 2 + 4])
    assert np.min(abs(before["return"].xi - xi)) > 1e-6
    requested = np.array([0, 0, -.03])
    records = drag(config, originals, prior, requested, xi=xi, mode="both", group="new")
    after = apply_corrections(originals, records, config)
    for key in after:
        assert xi in after[key].xi
        np.testing.assert_allclose(interpolate_at_xi(after[key], xi) - interpolate_at_xi(before[key], xi), requested, atol=2e-14)
    assert len({r.group_id for r in records if r.group_id == "new"}) == 1


def test_max_displacement_compares_correspondence_not_sampling_indices():
    times = np.array([0, 1, 2.])
    before = PathData("outbound", [[0, 0, 0], [.5, 0, 0], [1, 0, 0]], [0, .5, 1], times)
    after = PathData("outbound", [[0, 0, 0], [.25, 0, 0], [.5, .1, 0], [.75, 0, 0], [1, 0, 0]], [0, .25, .5, .75, 1], np.arange(5))
    assert max_corresponding_displacement({"outbound": before}, {"outbound": after}) == pytest.approx(.1)


def test_zero_drag_is_exact_memory_noop_and_weight_basis_frozen():
    config = Config()
    originals = generate_paths(config)
    first = drag(config, originals, [], [0, 0, .08])
    zero = drag(config, originals, first, [0, 0, 0], group="zero")
    assert [r.to_dict() for r in zero] == [r.to_dict() for r in first]
    weak = drag(config, originals, first, [0, .01, 0], group="same")
    strong = drag(config, originals, first, [0, .12, 0], group="same")
    np.testing.assert_array_equal(weak[-1].weight_xi, strong[-1].weight_xi)
    np.testing.assert_array_equal(weak[-1].weight_values, strong[-1].weight_values)
    before = apply_corrections(originals, first, config)["outbound"]
    s = arc_lengths(before.points)
    center_s = np.interp(.5, before.xi, s)
    np.testing.assert_allclose(weak[-1].weight_values, local_weights(np.interp(weak[-1].weight_xi, before.xi, s) - center_s, config.influence_length))


def test_tiny_second_drag_has_tiny_global_effect_and_preserves_old_shape():
    config = Config()
    originals = generate_paths(config)
    first = drag(config, originals, [], [0, 0, .08])
    before = apply_corrections(originals, first, config)
    second = drag(config, originals, first, [0, 0, .000001], group="tiny")
    after = apply_corrections(originals, second, config)
    assert max_corresponding_displacement(before, after) <= 1.000001e-6
    orthogonal = drag(config, originals, first, [0, .01, 0], xi=.55, group="orthogonal")
    result = apply_corrections(originals, orthogonal, config)
    for xi in (.35, .45, .65):
        before_z = interpolate_at_xi(before['outbound'], xi)[2]
        assert interpolate_at_xi(result['outbound'], xi)[2] == pytest.approx(before_z, abs=2e-14)


@pytest.mark.parametrize("change", [
    {"selection_radius_px": 0}, {"sample_dt": float("nan")}, {"base": [1, 2]},
    {"target": [0, 0, .12]}, {"unknown": 1}, {"gate": {"alpha": 1}},
    {"camera": {"up": [0, 0, 0]}}, {"colors": {"mode": "invalid"}},
])
def test_config_rejects_invalid_or_unknown_settings(change):
    with pytest.raises(ValueError):
        Config.from_dict(change)


def test_record_roundtrip_is_independent_and_strict():
    config = Config()
    record = drag(config, generate_paths(config), [], [0, 0, .08])[0]
    loaded = Correction.from_dict(record.to_dict())
    assert loaded.to_dict() == record.to_dict()
    loaded.delta[0] = 8
    assert record.delta[0] == 0
    data = record.to_dict()
    data["weight_values"] = [0] * len(data["weight_values"])
    with pytest.raises(ValueError, match="center"):
        Correction.from_dict(data)


def test_frozen_support_does_not_leak_past_influence_length_between_samples():
    config = Config(sample_dt=.2, influence_length=.137)
    originals = generate_paths(config)
    xi = .48
    records = drag(config, originals, [], [0, 0, .08], xi=xi)
    query = np.array([xi - config.influence_length / .6 - 1e-5,
                      xi + config.influence_length / .6 + 1e-5])
    np.testing.assert_array_equal(raw_field(originals['outbound'], records, config, query), np.zeros((2, 3)))

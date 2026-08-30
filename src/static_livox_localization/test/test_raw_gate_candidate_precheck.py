"""ROS-importable, no-node tests for the added pre-selection veto."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import trajectory_safety_gate as gate


def motion(v=0.0, w=0.0):
    return SimpleNamespace(valid=True, linear_speed_mps=v, angular_speed_rps=w)


def fixture_veto(monkeypatch, points=(), command=None, carried=None, age=0.1):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    monkeypatch.setattr(gate, 'collision_points_from_cloud', lambda cloud: points)
    return gate.make_raw_gate_candidate_veto(
        np.zeros((100, 3)), carried or motion(), age,
        command or (lambda v, w: (v, w, 0.0)))


def test_empty_obstacle_population_is_not_missing_sensor(monkeypatch):
    veto = fixture_veto(monkeypatch)
    assert not veto(0.35, 0.3)


def test_current_footprint_collision_rejects(monkeypatch):
    assert fixture_veto(monkeypatch, [(0.0, 0.0)] * 5)(0.35, 0.3)


def test_clear_nearly_straight_ramped_command_is_allowed(monkeypatch):
    veto = fixture_veto(monkeypatch, command=lambda v, w: (0.1, 0.03, 0.0))
    assert not veto(0.35, 0.5)


@pytest.mark.parametrize('age', [1.01, -0.1, float('nan')])
def test_invalid_cloud_age_rejects(monkeypatch, age):
    assert fixture_veto(monkeypatch, age=age)(0.35, 0.3)


def test_missing_cloud_or_motion_rejects():
    for cloud, observed in [(None, motion()), (np.zeros((99, 3)), motion()),
                            (np.zeros((100, 3)), None)]:
        veto = gate.make_raw_gate_candidate_veto(
            cloud, observed, 0.1, lambda v, w: (v, w, 0.0))
        assert veto(0.35, 0.3)


@pytest.mark.parametrize('v,w', [(float('nan'), 0.3), (0.3, float('inf')), (-0.1, 0.3)])
def test_invalid_command_rejects(monkeypatch, v, w):
    assert fixture_veto(monkeypatch)(v, w)


def test_carried_sweep_cannot_be_bypassed(monkeypatch):
    calls = []
    def collision(points, **kwargs):
        calls.append(kwargs)
        return kwargs['angular_speed_rps'] == -0.4
    monkeypatch.setattr(gate.base_gate, 'swept_footprint_collision', collision)
    veto = fixture_veto(monkeypatch, [(2.0, 0.5)] * 5, carried=motion(0.3, -0.4))
    assert veto(0.35, 0.3)
    assert len(calls) == 2


def test_collapsed_commands_are_cached(monkeypatch):
    calls = []
    def collision(points, **kwargs):
        calls.append(kwargs)
        return False
    monkeypatch.setattr(gate.base_gate, 'swept_footprint_collision', collision)
    veto = fixture_veto(monkeypatch, command=lambda v, w: (0.1, 0.1, 0.0))
    assert not veto(0.35, 0.3)
    assert not veto(0.35, 0.5)
    assert len(calls) == 1

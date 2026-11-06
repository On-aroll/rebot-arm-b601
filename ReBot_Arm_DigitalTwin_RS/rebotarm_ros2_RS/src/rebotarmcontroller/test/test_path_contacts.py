"""Contact-check equivalence on real models and threshold edges; no CAN."""
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from rebotarmcontroller.csp_twin_driver import CheckedKinematics


def test_real_model_matches_original_contact_predicate():
    kin = CheckedKinematics()
    data = mujoco.MjData(kin.collision_model)
    rng = np.random.default_rng(601)
    collisions = clear = 0
    for q in rng.uniform(kin.lower, kin.upper, size=(96, 6)):
        data.qpos[kin.qadr] = q
        mujoco.mj_forward(kin.collision_model, data)
        # Independent copy of the original per-contact body predicate.
        expected = any(contact.dist < -1e-5 and all(
            int(body) in kin.robot_bodies
            for body in kin.collision_model.geom_bodyid[[contact.geom1, contact.geom2]])
            for contact in data.contact)
        if expected:
            collisions += 1
            with pytest.raises(ValueError, match='自身碰撞'):
                kin.check_path(q, q, gravity_limits=np.full(6, 1e6))
        else:
            clear += 1
            kin.check_path(q, q, gravity_limits=np.full(6, 1e6))
    assert collisions and clear, 'Exercise both collision rejection and acceptance'


@pytest.mark.parametrize('distance,robot_pair,rejected', [
    (-1e-5 - 1e-12, True, True),
    (-1e-5, True, False),
    (-1e-5 + 1e-12, True, False),
    (-.01, False, False),
    (.01, True, False),
])
def test_contact_threshold_and_body_scope_preserved(monkeypatch, distance, robot_pair, rejected):
    kin = CheckedKinematics()
    robot = np.flatnonzero(kin.robot_geoms)
    outside = np.flatnonzero(~kin.robot_geoms)
    # Keep an unrelated contact first, so only inspecting the first contact
    # would miss the penetrating robot pair.
    contacts = SimpleNamespace(dist=np.array([-.01, distance]),
        geom1=np.array([outside[0], robot[0]]),
        geom2=np.array([robot[0], robot[1] if robot_pair else outside[0]]))
    kin.path_data.gravity = kin.model.createData()
    kin.path_data.collision = SimpleNamespace(qpos=np.zeros(kin.collision_model.nq), contact=contacts)
    monkeypatch.setattr(mujoco, 'mj_forward', lambda *_: None)
    if rejected:
        with pytest.raises(ValueError, match='自身碰撞'):
            kin.check_path(np.zeros(6), np.zeros(6), gravity_limits=np.full(6, 100.))
    else:
        kin.check_path(np.zeros(6), np.zeros(6), gravity_limits=np.full(6, 100.))


@pytest.mark.parametrize('degrees,expected', [(0., 7), (.05, 7), (8., 9)])
def test_every_original_path_sample_still_gets_full_collision_update(monkeypatch, degrees, expected):
    kin = CheckedKinematics()
    original = mujoco.mj_forward
    positions = []
    def forward(model, data):
        positions.append(data.qpos[kin.qadr].copy())
        original(model, data)
    monkeypatch.setattr(mujoco, 'mj_forward', forward)
    start = np.zeros(6)
    goal = start.copy(); goal[0] = np.deg2rad(degrees)
    kin.check_path(start, goal, gravity_limits=np.full(6, 100.))
    assert len(positions) == expected
    np.testing.assert_allclose(positions, np.linspace(start, goal, expected))

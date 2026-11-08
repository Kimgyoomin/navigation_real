import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_metrics_core import JOINTS, calculate, joint_powers, start_readiness_error


def test_start_with_10hz_clock_and_faster_sensor_headers():
    # Reproduce observed healthy sensors leading /clock by 88 / 84 ms.
    joints = [(10+i*.002, (1,)*6) for i in range(545)]
    odom = [(10+i*.004, (10+i*.004, 0)) for i in range(272)]
    assert start_readiness_error(joints, odom, 11., {'joint': .002, 'odom': .004}) is None
    # A valid bracket must survive until integration; no extrapolation.
    r = calculate(joints, odom, 11., 11.08)
    assert r['W_abs_J'] == pytest.approx(.48)


@pytest.mark.parametrize('case, expected', [
    ('missing', 'No valid'), ('paused', 'messages stopped'),
    ('future_clock_mismatch', 'mismatch'), ('stale_header', 'mismatch'),
    ('no_boundary_history', 'history'), ('uninitialized', 'not initialized')])
def test_start_rejects_invalid_inputs(case, expected):
    joints = [(10., (1,)*6), (10.09, (1,)*6)]
    odom = [(10., (0,0)), (10.08, (0,0))]
    ages = {'joint': .01, 'odom': .01}
    stamp = 10.
    if case == 'missing': joints = []
    if case == 'paused': ages['odom'] = 1.
    if case == 'future_clock_mismatch': joints[-1] = (10.4, (1,)*6)
    if case == 'stale_header': stamp = 11.
    if case == 'no_boundary_history': joints = [(10.05, (1,)*6), (10.09, (1,)*6)]
    if case == 'uninitialized': stamp = 0.
    assert expected in start_readiness_error(joints, odom, stamp, ages)


def test_mixed_sign_power_and_exact_boundaries():
    # Zero signed work must not cancel absolute work: 6 W for 0.8 s.
    joints = [(i/100, (1, -1, 1, -1, 1, -1)) for i in range(101)]
    odom = [(i/100, (i/100, 0)) for i in range(101)]
    r = calculate(joints, odom, .105, .905, mass=2, gravity=10)
    assert r['W_abs_J'] == pytest.approx(4.8)
    assert r['W_signed_J'] == pytest.approx(0)
    assert r['W_positive_J'] == pytest.approx(2.4)
    assert r['executed_length_xy_m'] == pytest.approx(.8)
    assert r['CoT_abs'] == pytest.approx(.3)


def test_named_joints_not_message_order():
    assert joint_powers(list(reversed(JOINTS)), [1]*6, list(range(6))) == (5,4,3,2,1,0)
    with pytest.raises(ValueError, match='missing_required_joint'):
        joint_powers(['other'], [1], [2])
    with pytest.raises(ValueError, match='nonfinite'):
        joint_powers(list(JOINTS), [float('nan')]*6, [1]*6)


@pytest.mark.parametrize('times, message', [([0, .1, .05, 1], 'nonmonotonic'),
                                         ([0, .1, 1], 'gap'),
                                         ([.2, .3], 'coverage')])
def test_bad_logs_never_produce_work(times, message):
    joints = [(t, (1,)*6) for t in times]
    odom = [(i/100, (i/100, 0)) for i in range(101)]
    with pytest.raises(ValueError, match=message):
        calculate(joints, odom, .1, .9)


def test_path_length_includes_detour_and_stops():
    joints = [(i/100, (1,)*6) for i in range(301)]
    odom = [(0, (0,0)), (1, (1,0)), (2, (1,1)), (3, (1,1))]
    r = calculate(joints, odom, 0, 3, odom_max_gap=1)
    assert r['executed_length_xy_m'] == pytest.approx(2)
    assert r['net_displacement_xy_m'] == pytest.approx(2**.5)
    assert r['W_abs_J'] == pytest.approx(18)

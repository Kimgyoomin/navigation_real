"""ROS-independent metrics. Time is simulation/header time; no extrapolation."""
import bisect
import math

JOINTS = ('L_HR_JOINT', 'L_HP_JOINT', 'L_KN_JOINT',
          'R_HR_JOINT', 'R_HP_JOINT', 'R_KN_JOINT')


def start_readiness_error(joints, odom, stamp, receipt_ages,
                          max_clock_skew=0.25, max_receipt_age=0.5):
    """Allow bounded header lead over a slower /clock, never missing history.

    Receipt ages use monotonic wall time so paused/stopped publishers cannot
    pass simply because simulation time has also stopped.
    """
    if not math.isfinite(stamp) or stamp <= 0:
        return 'Simulation clock is not initialized.'
    for label, samples in (('joint', joints), ('odom', odom)):
        if not samples:
            return f'No valid {label} samples received.'
        age = receipt_ages.get(label, math.inf)
        if not math.isfinite(age) or not 0 <= age <= max_receipt_age:
            return f'{label} messages stopped: receipt age={age:.3f}s; check simulation playback.'
        skew = stamp - samples[-1][0]
        if not math.isfinite(skew) or abs(skew) > max_clock_skew + 1e-9:
            return f'{label} header/clock mismatch: clock-header={skew:+.3f}s.'
        if samples[0][0] > stamp:
            return f'{label} history does not cover the start clock; wait for the next clock update.'
    return None


def joint_powers(names, effort, velocity):
    if len(names) != len(set(names)) or len(effort) != len(names) or len(velocity) != len(names):
        raise ValueError('joint_array_shape_or_duplicate_name')
    try:
        values = [(float(effort[names.index(j)]), float(velocity[names.index(j)])) for j in JOINTS]
    except ValueError as error:
        raise ValueError('missing_required_joint') from error
    if not all(math.isfinite(v) for pair in values for v in pair):
        raise ValueError('nonfinite_joint_sample')
    powers = tuple(torque * speed for torque, speed in values)
    if not all(math.isfinite(p) for p in powers):
        raise ValueError('nonfinite_joint_power')
    return powers


def clip_samples(samples, start, end, max_gap):
    """Samples are (timestamp, tuple); interpolate observables at exact bounds."""
    if end <= start or len(samples) < 2:
        raise ValueError('insufficient_samples_or_duration')
    times = [s[0] for s in samples]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('nonmonotonic_sample_time')
    if times[0] > start or times[-1] < end:
        raise ValueError('missing_boundary_coverage')
    lo = max(0, bisect.bisect_right(times, start) - 1)
    hi = min(len(times) - 1, bisect.bisect_left(times, end))
    data = samples[lo:hi+1]
    if any(b[0] - a[0] > max_gap + 1e-9 for a, b in zip(data, data[1:])):
        raise ValueError('sample_gap_exceeds_limit')

    def at(t):
        k = bisect.bisect_left(times, t)
        if times[k] == t:
            return samples[k]
        a, b = samples[k-1], samples[k]
        r = (t-a[0])/(b[0]-a[0])
        return (t, tuple(x + r*(y-x) for x, y in zip(a[1], b[1])))
    return [at(start)] + [s for s in data if start < s[0] < end] + [at(end)]


def calculate(joints, odom, start, end, mass=28.778101, gravity=9.8,
              joint_max_gap=0.05, odom_max_gap=0.10):
    if mass <= 0 or gravity <= 0:
        raise ValueError('invalid_mass_or_gravity')
    # Derive each non-negative power at native samples BEFORE interpolation.
    # |sum(tau*qdot)| is deliberately never used as absolute power.
    power = [(t, (sum(abs(p) for p in ps), sum(max(p, 0) for p in ps),
                   sum(max(-p, 0) for p in ps))) for t, ps in joints]
    power = clip_samples(power, start, end, joint_max_gap)
    position = clip_samples(odom, start, end, odom_max_gap)
    work = [sum((b[0]-a[0])*(a[1][i]+b[1][i])/2
                for a, b in zip(power, power[1:])) for i in range(3)]
    distance = sum(math.hypot(b[1][0]-a[1][0], b[1][1]-a[1][1])
                   for a, b in zip(position, position[1:]))
    net_distance = math.hypot(position[-1][1][0]-position[0][1][0],
                              position[-1][1][1]-position[0][1][1])
    duration = end-start
    rms = math.sqrt(sum((b[0]-a[0])*(a[1][0]**2+b[1][0]**2)/2
                        for a, b in zip(power, power[1:]))/duration)
    return dict(duration_s=duration, W_abs_J=work[0], W_positive_J=work[1],
                W_negative_magnitude_J=work[2], W_signed_J=work[1]-work[2],
                P_abs_mean_W=work[0]/duration, P_abs_rms_W=rms,
                P_abs_peak_W=max(p[1][0] for p in power),
                executed_length_xy_m=distance, net_displacement_xy_m=net_distance,
                CoT_abs=work[0]/(mass*gravity*distance) if distance > 1e-6 else None,
                CoT_net_displacement=work[0]/(mass*gravity*net_distance) if net_distance > 1e-6 else None,
                joint_samples=len(power), odom_samples=len(position))

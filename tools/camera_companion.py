"""Compose measured OAK RGB-D, attributed PX4 estimator state and CAM inference.

No hardware is opened and no policy or sender constructed at import. Device
startup must finish BEFORE OFFBOARD control; it is not serviced/bounded here.
run_measured_control uses caller-owned capture, runtime, EstimatorLink and receiver.
The ClockBinding translates PX4 boot seconds to the SAME host monotonic clock as
capture. A zero offset is not assumed. No simulator timestamp/target is needed.
"""
import argparse
import contextlib
import json
import math
import time


class MeasuredRgbdClient:
    """Bounded interface carrying measured frames on an explicitly named host clock."""
    def __init__(self, capture):
        self.capture, self.budget, self.failed = capture, None, False

    @contextlib.contextmanager
    def bounded_io(self, deadline, service, clock=time.monotonic):
        if self.failed or self.budget is not None:
            raise RuntimeError('measured RGB-D client failed or nested transaction')
        if not math.isfinite(deadline) or deadline <= clock() or not callable(service):
            raise ValueError('absolute deadline and bounded service callback required')
        self.budget = (deadline, service, clock)
        try:
            yield
            if clock() >= deadline:
                raise TimeoutError('measured RGB-D transaction deadline')
        except BaseException:
            self.failed = True
            raise
        finally:
            self.budget = None

    def frames(self):
        if self.failed or self.budget is None:
            raise RuntimeError('measured RGB-D frames require an active bounded transaction')
        deadline, service, clock = self.budget
        frame = self.capture.frames(deadline=deadline, service=service)
        if clock() >= deadline:
            raise TimeoutError('measured RGB-D frame deadline')
        return {'clock_now_s': frame['now_s'], 'rgb_t': frame['rgb_t'], 'depth_t': frame['depth_t'], 'frame': frame}


def run_measured_control(runtime, link, capture, *, max_steps, service, clock_binding,
                         altitude_m=1., clock=time.monotonic, sleep=time.sleep):
    """Already-ready segment; enforce armed/OFFBOARD and atomic receipt ages.

    EstimatorLink defaults to passive proposals. Publishing is a separate explicit
    caller choice. The receiver/service and stop response remain caller-owned.
    This does not arm, change modes, calibrate clocks or open USB. On any rejected
    tick runtime.invalidate() prevents silent reuse of advanced history.
    """
    from camera_estimator_link import EstimatorLink
    from sitl_camera import run_control
    if not isinstance(link, EstimatorLink):
        raise ValueError('an attributed atomic EstimatorLink is required')
    start_proposals = len(link.proposals)
    trace = run_control(runtime, link, MeasuredRgbdClient(capture), max_steps=max_steps,
                        service=service, clock_binding=clock_binding,
                        frame_adapter=lambda packet: packet['frame'], altitude_m=altitude_m,
                        clock=clock, sleep=sleep)
    proposed = link.proposals[start_proposals:]
    return {'trace': trace, 'commands_proposed': len(proposed),
            'commands_published': sum(bool(p['published']) for p in proposed),
            'clock': 'host_monotonic_via_explicit_measured_binding', 'hardware_qualification': 'NOT_TESTED'}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--check', required=True, help='prepared camera policy contract')
    ap.add_argument('--oak-config', help='optional actual measured camera calibration/evidence')
    args = ap.parse_args(argv)
    from camera_policy_contract import load_contract
    from oakd_camera import load_config, sdk_inventory
    contract = load_contract(args.check)
    if contract.get('lineage') == 'CAM-S2':
        # Runtime v1 (review r1 P3-3): the MAVLink companion is qualified for CAM-K2 v3 only.
        raise SystemExit('camera companion refused: CAM-S2 contracts are not supported (CAM-K2 v3 only)')
    measured = load_config(args.oak_config) if args.oak_config else None
    print(json.dumps({'policy_contract': 'VALID', 'seed': contract['seed'], **sdk_inventory(),
                      'measured_camera_config': 'VALID' if measured else 'NOT_PROVIDED',
                      'px4_clock_binding': 'NOT_PROVIDED', 'live_qualification': 'NOT_TESTED'}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

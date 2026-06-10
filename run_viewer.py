"""Interactive viewer: drive the ACTUATED shoulder hinge with a position reference.

Mechanism: fixed red ball -> arm 1 (actuated shoulder servo) -> arm 2 (passive
elbow). The PD position servo tracks the reference on the shoulder; the elbow
swings freely. Requires a display.

Usage:
    python run_viewer.py                      # default: 45 deg amplitude, 0.5 Hz
    python run_viewer.py --amp 60 --freq 0.25
    python run_viewer.py --hold 30            # constant 30 deg reference instead
"""

import argparse
import math
import time

import mujoco
import mujoco.viewer

MODEL_PATH = "model/single_hinge.xml"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Interactive hinge servo demo")
    p.add_argument("--amp", type=float, default=45.0, help="Sine amplitude (deg)")
    p.add_argument("--freq", type=float, default=0.5, help="Sine frequency (Hz)")
    p.add_argument("--hold", type=float, default=None,
                   help="Hold a constant reference angle (deg) instead of a sine")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    amp = math.radians(args.amp)
    hold = math.radians(args.hold) if args.hold is not None else None

    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        while viewer.is_running():
            step_start = time.time()

            # Reference signal -> servo target (ctrl is the target angle in rad)
            if hold is not None:
                data.ctrl[0] = hold
            else:
                data.ctrl[0] = amp * math.sin(2 * math.pi * args.freq * data.time)

            mujoco.mj_step(model, data)
            viewer.sync()

            # Real-time pacing
            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)

            # Print tracking error once per second
            if int(data.time) != int(data.time - model.opt.timestep):
                err = math.degrees(data.ctrl[0] - data.sensordata[0])
                print(f"t={data.time:6.1f}s  ref={math.degrees(data.ctrl[0]):7.2f} deg"
                      f"  shoulder={math.degrees(data.sensordata[0]):7.2f} deg"
                      f"  err={err:6.2f} deg"
                      f"  elbow(passive)={math.degrees(data.sensordata[3]):7.2f} deg")


if __name__ == "__main__":
    main()

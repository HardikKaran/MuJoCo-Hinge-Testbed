"""Headless step-response test for the ACTUATED shoulder joint of the 2-link arm.

Mechanism: fixed ball -> arm 1 (actuated shoulder) -> arm 2 (passive elbow).
Metrics are computed on the shoulder only; the passive elbow is logged and
plotted for reference.

Applies a step reference, logs joint state and actuator torque, computes
standard control metrics (rise time, overshoot, settling time, steady-state
error), and saves a 3-panel matplotlib figure to results/.

Usage:
    python run_step_test.py                          # 45 deg step, XML gains
    python run_step_test.py --step 60 --kp 40 --kv 4 # override gains
    python run_step_test.py --duration 5
"""

import argparse
import math
import os

import matplotlib

matplotlib.use("Agg")  # headless backend
import matplotlib.pyplot as plt
import mujoco
import numpy as np

MODEL_PATH = "model/single_hinge.xml"
RESULTS_DIR = "results"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Step-response test")
    p.add_argument("--step", type=float, default=45.0, help="Step angle (deg)")
    p.add_argument("--duration", type=float, default=3.0, help="Sim time (s)")
    p.add_argument("--kp", type=float, default=None, help="Override P gain")
    p.add_argument("--kv", type=float, default=None, help="Override D gain")
    p.add_argument("--tag", type=str, default="", help="Suffix for output filename")
    return p.parse_args()


def set_pd_gains(model: mujoco.MjModel, kp: float | None, kv: float | None) -> tuple[float, float]:
    """Override servo gains in-place. MuJoCo position actuator:
    gainprm[0] = kp, biasprm[1] = -kp, biasprm[2] = -kv."""
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "shoulder_servo")
    if kp is not None:
        model.actuator_gainprm[aid, 0] = kp
        model.actuator_biasprm[aid, 1] = -kp
    if kv is not None:
        model.actuator_biasprm[aid, 2] = -kv
    return model.actuator_gainprm[aid, 0], -model.actuator_biasprm[aid, 2]


def compute_metrics(t: np.ndarray, pos: np.ndarray, ref: float) -> dict:
    """Rise time (10-90%), overshoot %, settling time (2% band), SS error."""
    metrics = {}
    final = pos[-1]
    metrics["ss_error_deg"] = math.degrees(ref - final)

    # Rise time 10% -> 90% of reference
    try:
        t10 = t[np.argmax(pos >= 0.1 * ref)]
        t90 = t[np.argmax(pos >= 0.9 * ref)]
        metrics["rise_time_s"] = float(t90 - t10)
    except (ValueError, IndexError):
        metrics["rise_time_s"] = float("nan")

    # Overshoot relative to reference
    peak = pos.max() if ref > 0 else pos.min()
    metrics["overshoot_pct"] = float(100.0 * (peak - ref) / ref) if ref != 0 else 0.0

    # Settling time: last instant outside the +/-2% band
    band = 0.02 * abs(ref)
    outside = np.abs(pos - ref) > band
    metrics["settling_time_s"] = float(t[np.where(outside)[0][-1]]) if outside.any() else 0.0
    return metrics


def main() -> None:
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    kp, kv = set_pd_gains(model, args.kp, args.kv)
    ref = math.radians(args.step)
    n_steps = int(args.duration / model.opt.timestep)

    t = np.empty(n_steps)
    pos = np.empty(n_steps)
    vel = np.empty(n_steps)
    torque = np.empty(n_steps)
    elbow = np.empty(n_steps)

    data.ctrl[0] = ref
    for i in range(n_steps):
        mujoco.mj_step(model, data)
        t[i] = data.time
        pos[i], vel[i], torque[i], elbow[i] = data.sensordata[:4]

    m = compute_metrics(t, pos, ref)

    print(f"Step response: {args.step:.1f} deg | kp={kp:.1f}, kv={kv:.1f}")
    print(f"  rise time (10-90%) : {m['rise_time_s']*1000:7.1f} ms")
    print(f"  overshoot          : {m['overshoot_pct']:7.2f} %")
    print(f"  settling time (2%) : {m['settling_time_s']:7.3f} s")
    print(f"  steady-state error : {m['ss_error_deg']:7.3f} deg")

    # ---- Plot ----
    os.makedirs(RESULTS_DIR, exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)

    axes[0].plot(t, np.degrees(pos), label="shoulder (actuated)")
    axes[0].plot(t, np.degrees(elbow), color="tab:purple", lw=1,
                 label="elbow (passive)")
    axes[0].axhline(args.step, color="k", ls="--", lw=1, label="reference")
    axes[0].fill_between(t, args.step * 0.98, args.step * 1.02, alpha=0.15,
                         color="green", label="2% band")
    axes[0].set_ylabel("angle (deg)")
    axes[0].set_title(f"Step response — kp={kp:.0f}, kv={kv:.0f} | "
                      f"OS {m['overshoot_pct']:.1f}%, ts {m['settling_time_s']:.2f}s")
    axes[0].legend()

    axes[1].plot(t, np.degrees(vel), color="tab:orange")
    axes[1].set_ylabel("shoulder velocity (deg/s)")

    axes[2].plot(t, torque, color="tab:red")
    axes[2].set_ylabel("servo torque (N·m)")
    axes[2].set_xlabel("time (s)")

    for ax in axes:
        ax.grid(alpha=0.3)

    tag = f"_{args.tag}" if args.tag else ""
    out = os.path.join(RESULTS_DIR, f"step_{args.step:.0f}deg_kp{kp:.0f}_kv{kv:.0f}{tag}.png")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"  plot saved -> {out}")


if __name__ == "__main__":
    main()

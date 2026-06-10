# MuJoCo 2-Link Underactuated Arm Testbed

A planar 2-link arm for testing controlled actuation in MuJoCo:

```
fixed red ball (welded to world, mid-air)
   └─ shoulder hinge (ACTUATED, PD position servo)  ── arm 1
         └─ elbow hinge (PASSIVE, free + damping)   ── arm 2
```

Both hinges rotate about the world y-axis, so all motion is in the x-z
vertical plane. Joint angle 0 = links hanging straight down. The shoulder is
driven by a built-in PD position servo; the elbow swings freely — a
classic underactuated double pendulum.

## Project layout

```
mujoco-hinge-testbed/
├── model/
│   └── single_hinge.xml    # mechanism + actuator + sensors
├── run_viewer.py           # interactive viewer, sinusoidal/hold reference
├── run_step_test.py        # headless step test -> metrics + plots
├── results/                # plots land here
└── README.md
```

## Install

```bash
pip install mujoco matplotlib numpy
```

## Run

**Interactive viewer** (needs a display):

```bash
python run_viewer.py                   # 45 deg amplitude sine at 0.5 Hz on the shoulder
python run_viewer.py --amp 60 --freq 0.25
python run_viewer.py --hold 30         # hold a constant 30 deg shoulder reference
```

Prints shoulder reference/position/error and the passive elbow angle once
per second.

**Step-response test** (headless, saves plots to `results/`):

```bash
python run_step_test.py                              # 45 deg step, XML gains (kp=20, kv=2)
python run_step_test.py --step 60 --duration 6
python run_step_test.py --kp 60 --kv 6 --tag stiff   # gain sweep
```

Metrics (rise time 10–90%, overshoot, 2% settling time, steady-state error)
are computed on the **actuated shoulder only**; the passive elbow is logged
and shown on the position plot for reference.

## How the PD servo works

The `<position>` actuator in the XML *is* a PD controller implemented by
MuJoCo: torque `= K_p * (ctrl - q) - K_d * qdot`. `data.ctrl[0]` is the target shoulder angle in radians. `run_step_test.py` overrides gains at runtime via
`actuator_gainprm` / `actuator_biasprm`, so you can sweep gains without
touching the XML.

Sensors, in `data.sensordata` order: shoulder pos, shoulder vel, servo
torque, elbow pos.

## Things worth testing

- **Gravity load:** lifting the arm to +45 deg works against gravity, so the
  pure PD servo settles slightly short of the reference with a constant
  holding torque (~2 N·m at kp=60). Raise kp, or add gravity compensation
  via `data.qfrc_applied`, to kill the offset.
- **Passive dynamics:** step the shoulder and watch the elbow swing through,
  oscillate, and settle at minus-the-shoulder-angle (link 2 hangs vertical
  in the world frame). Increase elbow `damping` in the XML to change how
  fast it dies out.
- **Energy coupling:** drive the shoulder with `run_viewer.py` near the
  elbow's natural frequency and watch resonant swing-up — the starting point
  for classic underactuated control problems (e.g. Acrobot swing-up).
- **Torque saturation:** servo `forcerange` is ±30 N·m; large steps hit the
  limit (flat top in the torque plot).

## Model notes

- Timestep 2 ms, `implicitfast` integrator (good stability for stiff servos).
- The anchor ball is welded to the world, so MuJoCo's automatic parent-child
  contact exclusion does **not** apply there; the XML adds an explicit
  `<exclude body1="anchor" body2="arm1"/>`. The arm1-arm2 pair is joint-
  connected parent/child, so it is excluded automatically.
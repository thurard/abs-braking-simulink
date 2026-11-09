# ABS Braking — Modeling and Control in Simulink

Modeling and control of an **anti-lock braking system (ABS)** for a passenger
car, built in MATLAB/Simulink. The project builds the vehicle up from its
physical equations (braking force, car dynamics, wheel dynamics), shows why a
hard brake locks the wheels and lengthens the stopping distance, then designs
and compares several ABS controllers and an electronic brakeforce distribution
(EBD) strategy.

Individual project for the ES206 course (controlled mechatronic systems),
SeaTech (3rd year, SYSMER track).

## Problem

Under hard braking the driver asks for more force than the tyre can transmit.
The wheel decelerates faster than the car, the slip `g = (V - r·ω) / V` runs
away to 1 and the wheel locks. A locked wheel both lengthens the stopping
distance (the grip coefficient drops from its peak of ~0.95 near `g = 0.2` down
to ~0.6) and removes all steering control. The goal of the ABS is to hold the
slip near `g = 0.2`, where braking force is maximal, whatever the driver does.

## Physical model (part 2)

The car is modelled wheel by wheel and the loop is closed on the slip:

- **Braking block** — two regimes. Below slip, the road returns the demanded
  force, capped by Coulomb's law at `f·N = 1962 N`. Above slip, the force is set
  by the grip curve `F = f(g)·N`, with `f(g)` read from a 1-D lookup table. A
  `Switch` selects the regime at a slip threshold of 1e-3.
- **Car block** — `m·dV/dt = -4F`, a gain `-4/m` and an integrator, with the
  speed clamped at 0. Initial speed `v0 = 100 km/h = 27.77 m/s`.
- **Wheel block** — `J·dω/dt = r·f(g)·N - C`, where `C` is the brake torque and
  `r·f(g)·N` the road torque. This is where locking happens: once the brake
  torque exceeds what the road can transmit, the wheel locks in a few
  hundredths of a second, far faster than a driver can react.
- **Slip** computed from `V` and `ω` closes the loop into the braking and wheel
  blocks.
- **Pneumatic actuator** — the brake line behaves as a first-order lag
  `H(s) = 1 / (0.01·s + 1)` (τ = 10 ms), inserted on the demanded force.

Key parameters: `m = 1000 kg`, `N = 2452.5 N/wheel`, `r = 0.3 m`,
`J = 0.5 kg·m²`, `f_adh = 0.8`, `f_max ≈ 0.95` at `g ≈ 0.2`.

Without ABS, stopping distance is shortest just below the locking threshold
(`A ≈ 2330 N`): it drops from 98.6 m at 1000 N/wheel to ~43 m at 2300 N, then
jumps back to ~63 m once the wheel locks. The pneumatic lag changes the stopping
distance by less than 0.3 m.

## ABS controllers (part 3)

All controllers act through `F_command = min(F_driver, F_ABS)`, so the ABS is
transparent during gentle braking and only ever reduces the force.

| Controller | Principle | Stopping distance @ 2500 N |
| --- | --- | --- |
| None (locked wheel) | driver force applied directly | 62.66 m |
| Continuous PI | `Kp = 1e4`, `Ki = 1.5e5`, output 0–4000 N, anti-windup by clamping | 42.13 m |
| Discrete PI (10 kHz) | Euler integration, `Te = 0.1 ms`, zero-order hold | 42.13 m |
| Bang-bang (±400 N) | `F[k] = F[k-1] + 400·sign(0.2 - g[k])` | 41.97 m |
| Bang-bang + dead zone | three states increase / hold / decrease, `|e| ≤ 0.01` holds | 42.61 m |

The PI gains come from a linearised model around `g = 0.2` (the plant has an
unstable pole there, so Ziegler–Nichols does not apply): a unit-damping,
~30 rad/s closed loop below the 100 rad/s actuator cutoff. The discrete PI at
10 kHz is indistinguishable from the continuous one; dropping to 50 Hz makes the
loop oscillate at low speed, because the plant gain grows as `1/V`. The dead
zone trades a small distance penalty (+1.5 %) for far fewer valve switches,
mirroring a real hydraulic modulator's build / hold / release phases.

**ABS result:** stopping distance 62.66 m → 42.13 m at a 2500 N step
(−33 %), close to the 41.4 m theoretical optimum, and the wheels never lock, so
steering is kept throughout the stop.

## Electronic brakeforce distribution (part 4)

Braking shifts load onto the front axle (`ΔN = m·|a|·h / L`), so the lightly
loaded rear axle locks first. The model is split into front and rear axles with
dynamic normal loads (an algebraic loop broken by a 20 ms first-order filter on
the acceleration, which also stands in for suspension dynamics). EBD then shares
the demanded force in proportion to each axle's load
(`F_dem = A·N_axle / N0`), so both axles use the same fraction of their grip and
reach their limit together. At 2000 N/wheel this removes the premature rear
lock and recovers the 49.6 m stopping distance (vs 63.6 m without EBD).

## Repository contents

```
.
├── README.md
├── Code_ABS.m              # parameters, f(g) grip curve, EBD coefficients
├── Simulink_ABS.slx        # parts 2–3: physical model + ABS controllers
├── Simulink_ABS_Q4.slx     # part 4: load transfer + EBD
└── rapport_ABS.pdf         # full write-up (parts 2–4)
```

## How to run

1. Open MATLAB (R2024b or later) with Simulink.
2. Run `Code_ABS.m` to load the parameters and the grip curve into the
   workspace (it also prints the part 4 load-transfer checks).
3. Open `Simulink_ABS.slx` (parts 2–3) or `Simulink_ABS_Q4.slx` (EBD) and run
   the simulation; the scopes show the car and wheel speeds, the slip, and the
   demanded vs effective braking force.

## Context

SeaTech engineering school (Toulon, France), SYSMER track — marine and robotic
systems. ES206, controlled mechatronic systems.

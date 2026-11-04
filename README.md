# Two-Track Vehicle Model (MATLAB)

A two-track (four-wheel) vehicle dynamics model of a Formula Student car, built from first principles in MATLAB, with no toolboxes or prebuilt vehicle blocks.

Built alongside my work in the Southampton Formula Student vehicle dynamics sub-team.

## Why two-track?
A bicycle model lumps each axle into one tyre, so it can't see lateral load transfer. Tyre grip is non-linear with vertical load, so the way load moves between the inside and outside wheels sets the car's balance. A two-track model captures this, which makes it possible to study setup levers like the roll stiffness distribution (anti-roll bars).

## Status
| Stage | Description | Status |
|---|---|---|
| 1 | Vehicle parameters (`params.m`) | Done |
| 2 | Quasi-static load transfer: longitudinal, lateral (geometric + elastic split), aero (`loadTransfer.m`) | Done |
| 3 | Tyre model: simplified Pacejka with load sensitivity (`tyreLateral.m`) | Done |
| 4 | 3-DOF equations of motion: vx, vy, yaw rate, integrated with `ode45` (`vehicleODE.m`) | Done |
| 5 | Steady-state cornering: understeer gradient, max lateral g | In progress |
| 6 | Transient: step steer and sine steer | Planned |
| 7 | Validation against a bicycle model and team data | Planned |

## Usage
```matlab
test_loadTransfer   % sanity checks + corner loads for 4 manoeuvres
test_tyre           % tyre curves, load sensitivity, axle grip loss
test_vehicleODE     % straight-line, kinematic and symmetry checks + step steer
```

## Example result
Skidpad at 1.5 g, 15 m/s. Doubling the front roll stiffness (20 → 40 kNm/rad) cuts the inside front load from about 230 N to about 87 N. That shifts lateral load transfer forwards and adds understeer.

## Tyre model and load sensitivity
![Tyre curves](tyre_curves.png)

Peak friction falls as vertical load rises, so lateral load transfer costs an axle grip: the outside tyre gains less than the inside tyre loses. Axle grip lost at a 1.5 g skidpad:

| Front roll stiffness [Nm/rad] | Front grip loss | Rear grip loss |
|---|---|---|
| 10,000 | 3.8 % | 11.6 % |
| 20,000 | 7.6 % | 6.9 % |
| 40,000 | 12.4 % | 3.5 % |

Shifting roll stiffness forwards moves the grip loss to the front axle, which means more understeer. This is the mechanism behind balance tuning with anti-roll bars.

## Step steer (stage 4)
![Step steer](step_steer.png)

4° of steer ramped in over 0.1 s at 15 m/s. The car settles within about 0.3 s at 1.02 g on a 22.4 m radius. The geometric radius at this steer angle is L/δ = 22.2 m, so the car turns slightly less than geometry predicts: mild understeer, consistent with the near-equal front and rear grip losses above.

Checks: a straight line stays straight; the low-speed yaw rate matches the kinematic value v·δ/L to within 0.1%; left and right turns are exact mirror images; no wheel lifts during the manoeuvre.

**Current simplifications:** quasi-static load transfer (uses a_y ≈ v_x·r), no combined slip, no Ackermann, no roll or pitch dynamics.

## Notes
All parameters are placeholder estimates for a typical FS car and will be replaced with team data. SI units throughout. Axes: x forward, y left, z up.

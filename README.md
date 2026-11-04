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
| 3 | Tyre model (Pacejka with load sensitivity) | In progress |
| 4 | 3-DOF equations of motion (vx, vy, yaw rate) | Planned |
| 5 | Steady-state cornering: understeer gradient, max lateral g | Planned |
| 6 | Transient: step steer and sine steer | Planned |
| 7 | Validation against a bicycle model and team data | Planned |

## Usage
```matlab
test_loadTransfer   % runs sanity checks and prints corner loads for 4 manoeuvres
```

## Example result
Skidpad at 1.5 g, 15 m/s. Doubling the front roll stiffness (20 → 40 kNm/rad) cuts the inside front load from about 230 N to about 87 N. That shifts lateral load transfer forwards and adds understeer.

## Notes
All parameters are placeholder estimates for a typical FS car and will be replaced with team data. SI units throughout. Axes: x forward, y left, z up.

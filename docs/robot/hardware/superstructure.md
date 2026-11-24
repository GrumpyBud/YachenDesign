# Superstructure (2023 Charged Up), CAD revision 2026-11-24

Source: Onshape exports `Top Level Assembly Stowed.step`, `Top Level Assembly Intaking (roughly).step`, and `Top Level Assembly Scoring.step` (exported 2026-11-24 17:55 UTC). The three files have identical structure. Only part placements differ, so the joint motions come from the change in each part's placement between files (`tools/cad/steptree.py`, `tools/cad/pivot.py`). Nothing here has been measured on hardware.

Robot frame: +X forward (the claw side, CAD −Y), +Y left (CAD +X), +Z up from the floor (CAD Z). The front direction is an assumption: the claw and the elevator's lean both face CAD −Y.

## Mechanisms

| Mechanism | Hardware | Status |
| --- | --- | --- |
| Elevator | Two Kraken X60, 10t:38t spur stage, 24t HTD 5 mm pulley, WCP belt-in-tube continuous elevator, two moving stages (first stage, carriage) | CAD |
| Arm (claw pivot) | One Kraken X60 on a MAXPlanetary 5:1 + 3:1; pivot on the carriage | CAD; no further reduction found, provisional |
| Claw | Roller claw (3 in flex wheels, HTD belts). One Kraken X44, per the team; not in CAD yet | Motor confirmed by the team; reduction is an estimate |

## Facts

| Fact | Value | Status / source |
| --- | --- | --- |
| Elevator axis | 39.95° above horizontal, leaning forward | CAD (carriage motion direction, both poses agree) |
| Elevator gear ratio | 3.8:1 motor to pulley | CAD (10t/38t) |
| Elevator travel per pulley revolution | 0.120 m | CAD (24 × 5 mm); assumes the carriage moves 1:1 with the belt |
| Carriage travel inside the first stage | 0.609 m (24 in) | CAD: in the scoring pose the first stage had moved 0.699 m while the carriage had moved 1.308 m. In the intake pose the first stage had not moved |
| Elevator travel range | 0 to 1.32 m | 0 = CAD stow; 1.308 m = CAD score; the upper hard stop is unknown (provisional) |
| Arm pivot (elevator at 0) | x −0.0975 m, z 0.267 m | CAD (fixed point of the claw's rotation; intake and score agree) |
| Arm pivot to claw center | 0.541 m | CAD mesh centroid |
| Arm gear ratio | 15:1 | MAXPlanetary slices in CAD; final drive unknown (provisional) |
| Arm range | −30° to 100° | Provisional software limits; hard stops and collisions are unknown |
| Moving elevator mass | 8 kg | Estimate |
| Arm and claw mass | 3 kg | Estimate |
| CAN IDs, inversions, encoders | — | Unknown; no real IO is written yet |

## Positions

The arm angle is the elevation of the pivot-to-claw-center line above forward horizontal.

| Position | Elevator travel | Arm angle | Source |
| --- | --- | --- | --- |
| Stow | 0.000 m | 42.0° | CAD stowed |
| Intake | 0.224 m | −19.1° | CAD "Intaking (roughly)": the arm rotated 61.1° down |
| Score | 1.308 m | 16.5° | CAD scoring: the arm rotated 25.5° down |

Nothing in the CAD says which 2023 node "Score" targets, or how the robot avoids collisions while moving. The elevator and arm move at the same time.

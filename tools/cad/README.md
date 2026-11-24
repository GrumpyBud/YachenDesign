# CAD helpers

Use these scripts to regenerate the superstructure geometry and AdvantageScope models from Onshape STEP exports. Export the same top-level assembly once per robot pose (the files must have identical structure).

- `steptree.py STOW.step SCORE.step INTAKE.step out.json`: reads each file's assembly tree and part placements. It is pure Python with no dependencies.
- `pivot.py out.json tools/cad`: prints carriage travel, the arm rotation, and the pivot point for each pose.
- `step2glb.py STOW.step SCORE.step advantagescope/Robot_YachenDesign`: meshes the stowed export into `model.glb` (static base) plus `model_0.glb` (first stage), `model_1.glb` (carriage and pivot mount), and `model_2.glb` (arm and claw), in robot coordinates. It needs [`cadquery-ocp`](https://pypi.org/project/cadquery-ocp/) (`pip install cadquery-ocp` in a virtual environment).

`group_of()` in `step2glb.py` sorts each part by how it moves from the stowed export to the scoring export: no motion, about 0.7 m (first stage), about 1.3 m (carriage), or rotation (arm). Update those thresholds and `Choreographer.componentPoses()` if the mechanism changes.

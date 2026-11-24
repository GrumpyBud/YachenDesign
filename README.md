# Team 1466 TalonFX swerve

Java 25 project for SystemCore with WPILib 2027.0.0 Alpha 8, Commands v3, AdvantageKit 27.0.0 Alpha 7, and Phoenix 6 26.70.0 Alpha 2. The drivetrain is adapted from the [AdvantageKit 2026 TalonFX swerve template](https://github.com/Mechanical-Advantage/AdvantageKit/releases/tag/v26.0.2) using the [2027 AdvantageKit skeleton](https://github.com/Mechanical-Advantage/AdvantageKit/releases/tag/v27.0.0-alpha-7).

## Build, simulate, and replay

- Open this folder in WPILib VS Code with Java 25 selected. Run `.\gradlew.bat build` on Windows or `./gradlew build` on macOS or Linux.
- Use `spotlessApply` to format Java. The build checks formatting and runs `validateAutos`, which uses BLine to validate every path in the chooser.
- Run WPILib's **Simulate Robot Code** and select the GUI extension for interactive teleop. Desktop simulation uses Gradle's `run` task, publishes to NetworkTables, and writes WPILOG files to `logs/`.
- To replay a log, set `AKIT_LOG_PATH` to the absolute WPILOG path and run `.\gradlew.bat run` or `./gradlew run`. Leave HAL simulation extensions disabled. AdvantageKit writes the replay outputs to a sibling file with the `_sim` suffix. Clear the variable to resume ordinary simulation.
- The AdvantageScope metadata tab shows the packaged build date, Git commit, branch, and working-tree status. Copies outside a Git checkout report Git details as unknown.

In teleop, gamepad 0 uses the left stick for field-relative translation, right stick X for rotation, the west face button (Xbox X) for X-lock while held, and Back to make the current heading the driver's forward direction. The estimated field pose keeps its AprilTag reference.

## Elevator, arm, and claw (simulation only)

The 2023 Charged Up superstructure is a forward-tilted continuous elevator (two Kraken X60s), an arm pivoting on the carriage (one Kraken X60), and a roller claw (one Kraken X44). Its geometry and three positions come from the team's CAD exports. The [superstructure hardware record](docs/robot/hardware/superstructure.md) lists each value's source and which values are still estimates. Only simulation IO exists. On a real robot these mechanisms report as disconnected and never move.

| Gamepad 0 | Action |
| --- | --- |
| South face (Xbox A) | Stow |
| East face (Xbox B) | Intake position |
| North face (Xbox Y) | Score position |
| Right trigger (held) | Claw intake |
| Left trigger (held) | Claw eject |

Position buttons act in teleop only. The claw runs only while you hold a trigger, and after each enable you must release the triggers before they work again. The elevator and arm keep their goal through autonomous and disable. They go neutral while disabled, and after you enable they move back to the goal from wherever they are. In simulation the unpowered arm falls under gravity while disabled. With `TUNING_MODE` on, each position's elevator meters and arm degrees appear under `/Tuning/Choreographer`, and the joints' gains under `/Tuning/Elevator` and `/Tuning/Arm`.

### 3D view in AdvantageScope

1. In AdvantageScope, choose **Help > Use Custom Assets Folder** and select this repository's [`advantagescope`](advantagescope) folder.
2. Open a 3D Field tab, choose the **2023 Field**, and drag `RealOutputs/Odometry/Robot` onto it as a Robot. Choose the **YachenDesign 2023** model.
3. Drag `RealOutputs/Choreographer/ComponentPoses` onto that robot as its components. To see where the mechanisms are heading, add `GoalComponentPoses` as a ghost.

The models are meshed from the stowed CAD export with [`tools/cad`](tools/cad). Component 0 is the first stage, 1 the carriage and pivot mount, and 2 the arm and claw.

## Swerve characterization

Select Test (Utility) mode on the Driver Station and enable with both bumpers released. Use gamepad 0.

| Hold | Test | Result |
| --- | --- | --- |
| Left bumper | On carpet, the drive aligns the modules and turns in place for one measured revolution. Requires a connected Pigeon 2 and verified module locations and gearing. | Individual and mean effective wheel radius in meters. |
| Right bumper | With a clear straight lane ahead, the drive aligns forward and ramps up to 2.5 V at 0.4 V/s. Requires every drive module to use voltage control. | Individual and mean drive `kS` in volts and `kV` in volts per wheel rotation per second. |

Release the bumper to stop. Disable, controller loss, a detected sensor or alignment fault, or reaching the test limit also ends the run. Release both bumpers before starting another run. In AdvantageScope, look under `Drive/Characterization` for status, result validity, and estimates. The individual module inputs under `Drive/Module0` through `Drive/Module3` show measured voltage, speed, and travel. Review repeat runs before updating Tuner constants. Desktop simulation can exercise the feedforward sequence with the simulated motor model. The wheel-radius test requires an independent gyro measurement.

## Autonomous with BLine

Select **Auto Routine** in a WPILib 2027 dashboard with Selectable support before enabling autonomous. The starter uses [BLine 2027 Beta 3](https://docs.bline.run/getting-started/installation/). Do Nothing is the default.

| Routine | Sequence |
| --- | --- |
| Do Nothing | Hold the drivetrain idle. |
| Drive 1 Meter | Reset to the authored start, drive 1 m forward, and stop. |
| Strafe 1 Meter (Calibration) | Reset to the authored start, strafe 1 m left while holding heading, and stop. |
| Turn 90 Degrees (Calibration) | Reset to the authored start, rotate 90° counterclockwise in place, and stop. |

Motion examples share a blue-origin practice start of (2 m, 4 m, 0°). BLine transforms the path and its reset pose together for red. Each example stops on completion, disable, or a 5 s timeout. Use the calibration runs to compare measured travel and angle against field marks and review `Auto/FinalPose`, `Auto/Status`, and `FollowPath/*` in AdvantageScope. Odometry reports the estimated pose. Measure actual travel independently when calibrating.

To add a routine, export a path from [BLine Web](https://web-beta.bline-web.pages.dev/) into [autos/paths](src/main/deploy/autos/paths), then add one row in [AutoRoutines.java](src/main/java/first/robot/AutoRoutines.java) with its display name, filename without `.json`, and timeout in seconds. Keep the authored start as the first waypoint. Shared speed, acceleration, and endpoint tolerances live in [autos/config.json](src/main/deploy/autos/config.json). Vision and path flipping use the 2023 Charged Up layout, and BLine mirrors paths for red. The starter paths were authored for a generic practice start. Update the layout, alliance symmetry, and authored starts together for a different field.

Set `TUNING_MODE` in [Constants.java](src/main/java/first/robot/Constants.java) during tuning. AdvantageScope can edit the three BLine P gains under `/Tuning/BLine`. AdvantageKit records the edits and reports effective gains under `Auto/Tuning`. Use the same tuning-mode setting when replaying a run.

## Practice diagnostics

AdvantageScope shows the active commands under `Commands/Active`, loop phase durations under `LoggedTracer`, Choreographer goals and states, and drivetrain stator current under `Diagnostics/Drive`. The current total sums the logged motor measurements. The Driver Station alert reports low battery voltage after two seconds disabled.

## Vision

The [AdvantageKit vision template](https://docs.advantagekit.org/getting-started/template-projects/vision-template/) supplies camera IO, logged pose observations, ambiguity and field-boundary filtering, and distance-weighted estimator measurements. Desktop simulation uses one virtual PhotonVision camera driven by wheel-only simulated pose. Real camera entries start empty. Add each Luma P1's PhotonVision camera name and measured robot-to-camera transform in [VisionConstants.java](src/main/java/first/robot/subsystems/vision/VisionConstants.java). Match the coprocessor's PhotonVision build and field layout to the project. Review accepted and rejected samples under `Vision/Summary`.

## Real hardware

The drivetrain uses two Talon FX controllers and a CANcoder per module, plus a Pigeon 2. The checked-in [Tuner constants](src/main/java/first/robot/generated/TunerConstants.java) are upstream examples. Before enabling `ENABLE_REAL_DRIVE` in [Constants.java](src/main/java/first/robot/Constants.java), supply the robot's CAN bus, device IDs, encoder offsets and directions, gearing, module locations, wheel radius, speed, current limits, and gains. Check the Phoenix firmware against the [SystemCore Phoenix documentation](https://github.com/wpilibsuite/SystemcoreTesting/blob/main/CTR-Phoenix.md). With real drive disabled, the robot reports an alert and does not construct Phoenix devices.

Run `checkConstantsDeploy` to check deployment readiness locally. SystemCore deploys require real drive to be enabled and the autonomous paths to pass validation. On a branch whose name starts with `event`, a deploy stages and commits all changes Git finds in this checkout after those checks pass, then packages that commit's metadata. Use an event branch such as `event/week-1`. The commit is local.

The [robot context](docs/robot/context.md) and [FRC toolkit](AGENTS.md) describe project conventions.

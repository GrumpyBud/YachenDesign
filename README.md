# Team 1466 TalonFX swerve

Java 25 project for SystemCore with WPILib 2027.0.0 Alpha 8, Commands v3, AdvantageKit 27.0.0 Alpha 7, and Phoenix 6 26.70.0 Alpha 2. The drivetrain is adapted from the [AdvantageKit 2026 TalonFX swerve template](https://github.com/Mechanical-Advantage/AdvantageKit/releases/tag/v26.0.2) using the [2027 AdvantageKit skeleton](https://github.com/Mechanical-Advantage/AdvantageKit/releases/tag/v27.0.0-alpha-7).

## Build, simulate, and replay

- Open this folder in WPILib VS Code with Java 25 selected. Run `.\gradlew.bat build` on Windows or `./gradlew build` on macOS or Linux.
- Use `spotlessApply` to format Java. The build checks formatting and runs `validateAutos`, which uses BLine to validate every Java-defined and drawn path.
- Run WPILib's **Simulate Robot Code** and select the GUI extension for interactive teleop. Desktop simulation uses Gradle's `run` task, publishes to NetworkTables, and writes WPILOG files to `logs/`.
- To replay a log, set `AKIT_LOG_PATH` to the absolute WPILOG path and run `.\gradlew.bat run` or `./gradlew run`. Leave HAL simulation extensions disabled. AdvantageKit writes the replay outputs to a sibling file with the `_sim` suffix. Clear the variable to resume ordinary simulation.
- The AdvantageScope metadata tab shows the packaged build date, Git commit, branch, and working-tree status. Copies outside a Git checkout report Git details as unknown.

In teleop, gamepad 0 uses the left stick for field-relative translation, right stick X for rotation, the west face button (Xbox X) for X-lock while held, and Back to make the current heading the driver's forward direction. The estimated field pose keeps its AprilTag reference.

## Swerve characterization

Select Test (Utility) mode on the Driver Station and enable with both bumpers released. Use gamepad 0.

| Hold | Test | Result |
| --- | --- | --- |
| Left bumper | On carpet, the drive aligns the modules and turns in place for one measured revolution. Requires a connected Pigeon 2 and verified module locations and gearing. | Individual and mean effective wheel radius in meters. |
| Right bumper | With a clear straight lane ahead, the drive aligns forward and ramps up to 2.5 V at 0.4 V/s. Requires every drive module to use voltage control. | Individual and mean drive `kS` in volts and `kV` in volts per wheel rotation per second. |

Release the bumper to stop. Disable, controller loss, a detected sensor or alignment fault, or reaching the test limit also ends the run. Release both bumpers before starting another run. In AdvantageScope, look under `Drive/Characterization` for status, result validity, and estimates. The individual module inputs under `Drive/Module0` through `Drive/Module3` show measured voltage, speed, and travel. Review repeat runs before updating Tuner constants. Desktop simulation can exercise the feedforward sequence with the simulated motor model. The wheel-radius test requires an independent gyro measurement.

## Autonomous with BLine

Select **Auto Routine** in a WPILib 2027 dashboard with Selectable support before enabling autonomous. The starter uses [BLine 2027 Beta 3](https://docs.bline.run/getting-started/installation/). Do Nothing is the default.

Autos come from two places, and both appear in the same chooser:

- **Java routines (the normal way).** Define paths as `public static Path name()` methods in [AutoPaths.java](src/main/java/first/robot/auto/AutoPaths.java) and sequence them in [Autos.java](src/main/java/first/robot/auto/Autos.java). A routine can follow several paths, wait, run mechanism commands, wait for a sensor, and stop with a fault when a step fails:

  ```java
  new AutoRoutine("Two Piece", 15, auto -> {
    if (!auto.follow(AutoPaths.toFirstPiece())) return;
    if (!auto.await(intake.collect())) return;
    auto.follow(AutoPaths.backToScore());
  })
  ```

- **Drawn paths (competition-day changes).** Export a path from [BLine Web](https://web-beta.bline-web.pages.dev/) into [autos/paths](src/main/deploy/autos/paths) and deploy. It appears as **Drawn: name** with no code change and a 15 s limit. A Java routine can also use a drawn path with `auto.followDrawn("name")`.

| Routine | Source | Sequence |
| --- | --- | --- |
| Do Nothing | Built in | Hold the drivetrain idle. |
| Drive 1 Meter | Java | Reset to the authored start, drive 1 m forward, and stop. |
| Strafe 1 Meter (Calibration) | Java | Reset to the authored start, strafe 1 m left while holding heading, and stop. |
| Turn 90 Degrees (Calibration) | Java | Reset to the authored start, rotate 90° counterclockwise in place, and stop. |
| Example: Out and Back | Java | Drive 1 m forward, pause 0.5 s, return to the start at 0.5 m/s. |
| Drawn: drive-1m | Drawn | Same motion as Drive 1 Meter, from `drive-1m.json`. |

Write paths in blue-origin meters. BLine flips the path and its reset pose together for red. The first path a routine follows resets odometry to its authored start, so it must begin with a `Waypoint`. Later paths start from the measured pose. Every path, Java or drawn, uses the shared speed, acceleration, and endpoint tolerances in [autos/config.json](src/main/deploy/autos/config.json) unless it sets its own `PathConstraints`. A routine stops on completion, a failed step, disable, or its timeout. A failed step or timeout raises the **AutonomousFault** alert and reports the reason in `Auto/Status`. Use the calibration runs to compare measured travel and angle against field marks and review `Auto/FinalPose`, `Auto/Status`, `Auto/Segment`, and `FollowPath/*` in AdvantageScope. Odometry reports the estimated pose. Measure actual travel independently when calibrating.

Path events (`EventTrigger` in Java, `event_trigger` in drawn paths) run actions registered in `Autos.registerEvents()`. BLine only logs a warning for an unregistered name. Event commands keep running after their path ends, so give them their own end condition.

`validateAutos` checks every `AutoPaths` method and every drawn file during the build, and it requires each drawn path to start with a waypoint. `AutoRoutinesSimTest` runs every chooser routine against the simulated swerve, including a red-alliance flip. That shows the steps happen in the right order. It does not show how the real robot drives on carpet. Vision and path flipping use the 2023 Charged Up layout, and BLine mirrors paths for red. The starter paths were authored for a generic practice start. Update the layout, alliance symmetry, and authored starts together for a different field.

Set `TUNING_MODE` in [Constants.java](src/main/java/first/robot/Constants.java) during tuning. AdvantageScope can edit the three BLine P gains under `/Tuning/BLine`. AdvantageKit records the edits and reports effective gains under `Auto/Tuning`. Use the same tuning-mode setting when replaying a run.

## Practice diagnostics

AdvantageScope shows the active commands under `Commands/Active`, loop phase durations under `LoggedTracer`, Choreographer goals and states, and drivetrain stator current under `Diagnostics/Drive`. The current total sums the logged motor measurements. The Driver Station alert reports low battery voltage after two seconds disabled.

## Vision

The [AdvantageKit vision template](https://docs.advantagekit.org/getting-started/template-projects/vision-template/) supplies camera IO, logged pose observations, ambiguity and field-boundary filtering, and distance-weighted estimator measurements. Desktop simulation uses one virtual PhotonVision camera driven by wheel-only simulated pose. Real camera entries start empty. Add each Luma P1's PhotonVision camera name and measured robot-to-camera transform in [VisionConstants.java](src/main/java/first/robot/subsystems/vision/VisionConstants.java). Match the coprocessor's PhotonVision build and field layout to the project. Review accepted and rejected samples under `Vision/Summary`.

## Real hardware

The drivetrain uses two Talon FX controllers and a CANcoder per module, plus a Pigeon 2. The checked-in [Tuner constants](src/main/java/first/robot/generated/TunerConstants.java) are upstream examples. Before enabling `ENABLE_REAL_DRIVE` in [Constants.java](src/main/java/first/robot/Constants.java), supply the robot's CAN bus, device IDs, encoder offsets and directions, gearing, module locations, wheel radius, speed, current limits, and gains. Check the Phoenix firmware against the [SystemCore Phoenix documentation](https://github.com/wpilibsuite/SystemcoreTesting/blob/main/CTR-Phoenix.md). With real drive disabled, the robot reports an alert and does not construct Phoenix devices.

Run `checkConstantsDeploy` to check deployment readiness locally. SystemCore deploys require real drive to be enabled and the autonomous paths to pass validation. On a branch whose name starts with `event`, a deploy stages and commits all changes Git finds in this checkout after those checks pass, then packages that commit's metadata. Use an event branch such as `event/week-1`. The commit is local.

The [robot context](docs/robot/context.md) and [FRC toolkit](AGENTS.md) describe project conventions.

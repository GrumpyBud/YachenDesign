# Team 1466 TalonFX swerve

Java 25 project for SystemCore with WPILib 2027.0.0 Alpha 8, Commands v3, AdvantageKit 27.0.0 Alpha 7, and Phoenix 6 26.70.0 Alpha 2. The drivetrain is adapted from the [AdvantageKit 2026 TalonFX swerve template](https://github.com/Mechanical-Advantage/AdvantageKit/releases/tag/v26.0.2) using the [2027 AdvantageKit skeleton](https://github.com/Mechanical-Advantage/AdvantageKit/releases/tag/v27.0.0-alpha-7).

## Build, simulate, and replay

- Open this folder in WPILib VS Code with Java 25 selected. Run `.\gradlew.bat build` on Windows or `./gradlew build` on macOS or Linux.
- Use `spotlessApply` to format Java. The build checks formatting and runs `validateAutos`, which uses BLine to validate every Java-defined and drawn path.
- Run WPILib's **Simulate Robot Code** and select the GUI extension for interactive teleop. Desktop simulation uses Gradle's `run` task, publishes to NetworkTables, and writes WPILOG files to `logs/`.
- For quick iteration on Windows, run **Terminal → Run Task → Sim: run and restart on save** (or `.\tools\sim\watch.ps1`, with `-Gui` for the WPILib sim GUI and `-DriverStation` to let the real Driver Station connect). It builds, starts the simulator, and rebuilds and restarts it about 2 to 3 seconds after you save anything under `src/main`, `build.gradle`, or `vendordeps`. If the build fails, the previous simulator keeps running and the error is shown. Press `r` to restart or `q` to quit. AdvantageScope and Elastic reconnect by themselves. Stop any other simulator first; the script refuses to start while port 5810 is in use.
- To replay a log, set `AKIT_LOG_PATH` to the absolute WPILOG path and run `.\gradlew.bat run` or `./gradlew run`. Leave HAL simulation extensions disabled. AdvantageKit writes the replay outputs to a sibling file with the `_sim` suffix. Clear the variable to resume ordinary simulation.
- The AdvantageScope metadata tab shows the packaged build date, Git commit, branch, and working-tree status. Copies outside a Git checkout report Git details as unknown.

In teleop, gamepad 0 uses the left stick for field-relative translation, right stick X for rotation, the west face button (Xbox X) for X-lock while held, and Back to make the current heading the driver's forward direction. The estimated field pose keeps its AprilTag reference.

## Swerve characterization

Select Utility mode on the Driver Station and enable with both bumpers released. Use gamepad 0.

| Hold | Test | Result |
| --- | --- | --- |
| Left bumper | On carpet, the drive aligns the modules and turns in place for one measured revolution. Requires a connected Pigeon 2 and verified module locations and gearing. | Individual and mean effective wheel radius in meters. |
| Right bumper | With a clear straight lane ahead, the drive aligns forward and ramps up to 2.5 V at 0.4 V/s. Requires every drive module to use voltage control. | Individual and mean drive `kS` in volts and `kV` in volts per wheel rotation per second. |

Release the bumper to stop. Disable, controller loss, a detected sensor or alignment fault, or reaching the test limit also ends the run. Release both bumpers before starting another run. In AdvantageScope, look under `Drive/Characterization` for status, result validity, and estimates. The individual module inputs under `Drive/Module0` through `Drive/Module3` show measured voltage, speed, and travel. Review repeat runs before updating Tuner constants. Desktop simulation can exercise the feedforward sequence with the simulated motor model. The wheel-radius test requires an independent gyro measurement.

## Elastic dashboard

The layout lives in `src/main/deploy/elastic-layout.json` and deploys with the code. The robot serves its deploy folder on port 5800, in simulation too. In Elastic, connect to the robot (or `localhost` in simulation), then use **File → Download From Robot** and pick `elastic-layout`. It has three tabs:

- **Autonomous**: the routine and scoring-plan choosers, whether the claw has a piece and the mechanism is in position, the running plan's status, pickup and result, and alerts.
- **Teleoperated**: Choreographer state and position, mechanism positions, claw current, and alerts.
- **Utility**: the characterization status and its wheel radius and drive `kS`/`kV` results.

If you rearrange it in Elastic, save it over `src/main/deploy/elastic-layout.json` and commit it so every laptop gets the same dashboard.

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
| 2023: Custom (dashboard) | Java | A scoring plan from the dashboard choosers below. |
| 2023: Node 8, Piece 4, Node 7 | Java | Score the preload on node 8, pick up staged piece 4 (loading zone side), score it on node 7. About 10.5 s in simulation. |
| 2023: Node 8, Piece 4 (fallback 3), Node 7 | Java | As above; if piece 4 is not detected, try piece 3. About 14.4 s in simulation when the fallback is needed. |
| 2023: Node 2, Piece 1 (fallback 2), Node 3 | Java | Bump side through the lower lane, with piece 2 as the fallback. About 10.5 s, or 14.3 s with the fallback. |
| Drive 1 Meter | Java | Reset to the authored start, drive 1 m forward, and stop. |
| Strafe 1 Meter (Calibration) | Java | Reset to the authored start, strafe 1 m left while holding heading, and stop. |
| Turn 90 Degrees (Calibration) | Java | Reset to the authored start, rotate 90° counterclockwise in place, and stop. |
| Example: Out and Back | Java | Drive 1 m forward, pause 0.5 s, return to the start at 0.5 m/s. |
| Drawn: drive-1m | Drawn | Same motion as Drive 1 Meter, from `drive-1m.json`. |

Write paths in blue-origin meters. BLine flips the path and its reset pose together for red. The first path a routine follows resets odometry to its authored start, so it must begin with a `Waypoint`. Later paths start from the measured pose. Every path, Java or drawn, uses the shared speed, acceleration, and endpoint tolerances in [autos/config.json](src/main/deploy/autos/config.json) unless it sets its own `PathConstraints`. A routine stops on completion, a failed step, disable, or its timeout. A failed step or timeout raises the **AutonomousFault** alert and reports the reason in `Auto/Status`. Use the calibration runs to compare measured travel and angle against field marks and review `Auto/FinalPose`, `Auto/Status`, `Auto/Segment`, and `FollowPath/*` in AdvantageScope. Odometry reports the estimated pose. Measure actual travel independently when calibrating.

On red, `AutoContext` flips each path with the configured field symmetry. BLine's own flip always rotates about the field center, so for a mirrored field such as 2023 it also requests BLine's width mirror, giving (length - x, y, 180 deg - heading).

### 2023 scoring plans

Scoring autos are described by field locations, not drawn paths. Nodes are numbered 1 to 9 and staged pieces 1 to 4, both counting from the cable bump side; nodes 2, 5, and 8 are cubes. The numbers mean the same physical positions on either alliance. For **2023: Custom (dashboard)**, set these choosers before enabling autonomous: **Auto Start Node**, **Auto Pickup**, **Auto Score Node**, **Auto Fallback 1**, and **Auto Fallback 2**. The choices are logged, so a replay uses the same plan. Add a fixed plan in `Autos.java` with `scoring(robot, name, new ScoringPlan(start, piece, score, List.of(fallbacks)))`.

`CyclePaths` builds every leg from `ChargedUpField` geometry (Team 6328's 2023 FieldConstants), and the build validates all 128 legs any plan can produce. The robot is tippy, so each plan follows these rules:

- The elevator and arm are fully stowed, and confirmed, before every drive.
- Mechanisms extend only as a leg slows to its stop: the intake lowers 0.6 m before stopping behind a piece, and the elevator raises 0.4 m before stopping at a node. Driving onto a piece happens slowly with the claw at the low intake position.
- The robot travels through a lane above or below the charge station, whichever side the trip is on, and never crosses the station. It turns only east of the station, never in a lane.
- Corners sharper than 35 degrees are approached at no more than 1.5 m/s, because BLine slows only for the final stop.

If the claw does not detect the piece, the routine tries each fallback in order while there is time. After a pickup it estimates the return time from the plan's paths. If the robot cannot score before 15 s, it stays retracted and keeps the piece for teleop (`Auto/Result`). If no piece is detected, the routine reports a fault. `Auto/PickupStatus` shows each attempt.

The claw detects a game piece from motor current: while intaking, stator current stays above `Claw/PieceCurrentAmps` with the rollers nearly stopped, after ignoring spin-up. Ejecting clears it. See `Claw/HasGamePiece`. The thresholds are estimates until measured with real game pieces. In simulation, `SimGamePieces` places the 2023 staged pieces (logged as `SimField/StagedPieces` and `SimField/HeldPiece` for AdvantageScope) and jams the rollers when the claw picks one up.

Path events (`EventTrigger` in Java, `event_trigger` in drawn paths) run actions registered in `Autos.registerEvents()`. The registered events are `stow`, `intake` (claw down, rollers on), and `score`. Routines, events, and driver controls move mechanisms through `Choreographer.requestPosition(source, ...)` and `requestClaw(source, ...)`. The Choreographer obeys only the source that owns the current mode, `DRIVER` in teleop and `AUTO` in autonomous, and stops the claw whenever ownership changes. BLine only logs a warning for an unregistered name. Event commands keep running after their path ends, so give them their own end condition.

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

package first.robot.auto;

import static org.wpilib.units.Units.Seconds;

import bline.commands.FollowPath;
import bline.field.FlippingUtil;
import bline.following.DriveType;
import bline.path.Path;
import first.robot.Constants;
import first.robot.subsystems.Choreographer;
import first.robot.subsystems.Choreographer.Source;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.vision.VisionConstants;
import first.robot.util.LoggedTunableNumber;
import java.io.File;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.controller.PIDController;
import org.wpilib.system.Filesystem;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Runs the selected autonomous routine. Routines come from two places:
 *
 * <ul>
 *   <li>Java routines in {@link Autos#routines}, built from {@link AutoPaths} and scoring plans.
 *   <li>Every path drawn in BLine Web and exported to {@code deploy/autos/paths/}, listed as
 *       "Drawn: name" with no code changes.
 * </ul>
 */
public final class AutoRoutines {
  /** Drawn paths have no routine of their own, so they get one whole-auto limit. */
  static final double DRAWN_PATH_TIMEOUT_SECONDS = 15;

  static final String DRAWN_PREFIX = "Drawn: ";
  private static final AutoRoutine DO_NOTHING = new AutoRoutine("Do Nothing", 1, auto -> {});

  /** Gradle entry point for checking every Java and drawn path without starting a robot. */
  public static void main(String[] args) {
    var autosDirectory = new File(args[0]);
    Path.setProjectDefaults(Path.loadProjectDefaults(autosDirectory));
    var errors = new ArrayList<String>();

    var names = new HashSet<String>(List.of(DO_NOTHING.name()));
    try {
      for (var routine : Autos.routines(null, null)) {
        if (!names.add(routine.name())) {
          errors.add("Duplicate routine name: " + routine.name());
        }
      }
    } catch (RuntimeException error) {
      errors.add("Autos.routines(): " + error.getMessage());
    }

    // Every leg a scoring plan can generate, for any node and staged piece.
    for (var leg : CyclePaths.allLegs().entrySet()) {
      try {
        checkPath(leg.getValue());
      } catch (RuntimeException error) {
        errors.add("CyclePaths." + leg.getKey() + ": " + error.getMessage());
      }
    }
    System.out.println("Validated " + CyclePaths.allLegs().size() + " scoring plan legs");

    for (Method method : javaPathMethods()) {
      String label = "AutoPaths." + method.getName() + "()";
      try {
        checkPath((Path) method.invoke(null));
        System.out.println("Validated " + label);
      } catch (ReflectiveOperationException | RuntimeException error) {
        var cause = error.getCause() != null ? error.getCause() : error;
        errors.add(label + ": " + cause.getMessage());
      }
    }

    for (String name : drawnPathNames(autosDirectory)) {
      if (!names.add(DRAWN_PREFIX + name)) {
        errors.add("Duplicate routine name: " + DRAWN_PREFIX + name);
      }
      try {
        var path = new Path(autosDirectory, name);
        checkPath(path);
        if (path.getAuthoredStartPose().isEmpty()) {
          throw new IllegalArgumentException("A drawn auto must start with a waypoint");
        }
        System.out.println("Validated drawn path " + name + ".json");
      } catch (RuntimeException error) {
        errors.add(name + ".json: " + error.getMessage());
      }
    }

    errors.forEach(System.err::println);
    if (!errors.isEmpty()) {
      throw new IllegalStateException("Fix the autonomous paths listed above before continuing.");
    }
  }

  private static void checkPath(Path path) {
    if (!path.isValid()) {
      throw new IllegalArgumentException("Invalid path");
    }
  }

  private static List<Method> javaPathMethods() {
    return Arrays.stream(AutoPaths.class.getDeclaredMethods())
        .filter(m -> Modifier.isPublic(m.getModifiers()) && Modifier.isStatic(m.getModifiers()))
        .filter(m -> m.getParameterCount() == 0 && m.getReturnType() == Path.class)
        .sorted((a, b) -> a.getName().compareTo(b.getName()))
        .toList();
  }

  static List<String> drawnPathNames(File autosDirectory) {
    var files = new File(autosDirectory, "paths").listFiles((dir, name) -> name.endsWith(".json"));
    if (files == null) {
      return List.of();
    }
    return Arrays.stream(files)
        .map(file -> file.getName().substring(0, file.getName().length() - ".json".length()))
        .sorted()
        .toList();
  }

  @AutoLog
  static class AutoInputs {
    public boolean enabled;
    public boolean redAlliance;
  }

  private final Drive drive;
  private final Choreographer robot;
  private AutoOptions options;
  private final File autosDirectory = new File(Filesystem.getDeployDirectory(), "autos");
  private final LoggedNetworkChooser<AutoRoutine> chooser =
      new LoggedNetworkChooser<>("Auto Routine");
  private final Scheduler scheduler = Scheduler.getDefault();
  private final AutoInputsAutoLogged inputs = new AutoInputsAutoLogged();
  private final LoggedTunableNumber translationKp =
      new LoggedTunableNumber("BLine/TranslationKp", 2.5);
  private final LoggedTunableNumber rotationKp = new LoggedTunableNumber("BLine/RotationKp", 4);
  private final LoggedTunableNumber crossTrackKp =
      new LoggedTunableNumber("BLine/CrossTrackKp", 2.0);
  private final PIDController translationController = new PIDController(0, 0, 0);
  private final PIDController rotationController = new PIDController(0, 0, 0);
  private final PIDController crossTrackController = new PIDController(0, 0, 0);
  private final FollowPath.Builder follower;
  private final Alert fault = new Alert("AutonomousFault", "Autonomous stopped.", Level.HIGH);
  private final Alert configFault =
      new Alert("AutonomousConfig", "Autonomous paths unavailable.", Level.HIGH);
  private Command active;
  private boolean previouslyEnabled;
  private boolean pathFinished;
  private String lastStatus = "Idle";
  private final List<AutoRoutine> routines = new ArrayList<>();

  public AutoRoutines(Drive drive, Choreographer robot) {
    this.drive = drive;
    this.robot = robot;

    // Java paths use config.json's speed and tolerance defaults, the same as drawn paths.
    try {
      Path.setProjectDefaults(Path.loadProjectDefaults(autosDirectory));
    } catch (RuntimeException error) {
      configFault.setText("Could not load autos/config.json: " + error.getMessage());
      configFault.set(true);
    }

    chooser.addDefault(DO_NOTHING.name(), DO_NOTHING);
    options = new AutoOptions();
    routines.addAll(Autos.routines(robot, options));
    for (String name : drawnPathNames(autosDirectory)) {
      routines.add(
          new AutoRoutine(
              DRAWN_PREFIX + name, DRAWN_PATH_TIMEOUT_SECONDS, auto -> auto.followDrawn(name)));
    }
    routines.forEach(routine -> chooser.add(routine.name(), routine));
    Autos.registerEvents(robot);

    var field = VisionConstants.aprilTagLayout;
    FlippingUtil.fieldSizeX = field.getFieldLength();
    FlippingUtil.fieldSizeY = field.getFieldWidth();
    // 2023 Charged Up mirrors the red alliance across the field center line.
    FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kMirrored;
    follower =
        new FollowPath.Builder(
                DriveType.SWERVE,
                drive,
                drive::getPose,
                drive::setAutoStartPose,
                drive::getChassisVelocities,
                drive::runVelocity,
                translationController,
                rotationController,
                crossTrackController)
            .withShouldFlip(() -> inputs.redAlliance);

    FollowPath.setPoseLoggingConsumer(
        entry -> Logger.recordOutput(entry.getFirst(), entry.getSecond()));
    FollowPath.setTranslationListLoggingConsumer(
        entry -> Logger.recordOutput(entry.getFirst(), entry.getSecond()));
    FollowPath.setDoubleLoggingConsumer(
        entry -> Logger.recordOutput(entry.getFirst(), entry.getSecond()));
    FollowPath.setBooleanLoggingConsumer(
        entry -> {
          Logger.recordOutput(entry.getFirst(), entry.getSecond());
          if (entry.getFirst().equals("FollowPath/finished")) {
            pathFinished = entry.getSecond();
          }
        });
  }

  public boolean updateInputs() {
    if (Constants.currentMode != Constants.Mode.REPLAY) {
      inputs.enabled = RobotState.isAutonomousEnabled();
      inputs.redAlliance = MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
    }
    Logger.processInputs("Auto/Mode", inputs);
    return inputs.enabled;
  }

  public void periodic(boolean ownsDrive) {
    translationController.setP(Math.max(0, translationKp.get()));
    rotationController.setP(Math.max(0, rotationKp.get()));
    crossTrackController.setP(Math.max(0, crossTrackKp.get()));
    Logger.recordOutput("Auto/Tuning/TranslationKp", translationController.getP());
    Logger.recordOutput("Auto/Tuning/RotationKp", rotationController.getP());
    Logger.recordOutput("Auto/Tuning/CrossTrackKp", crossTrackController.getP());

    if (ownsDrive && !previouslyEnabled) {
      start();
    } else if (!ownsDrive && previouslyEnabled && active != null) {
      if (scheduler.isScheduledOrRunning(active)) {
        scheduler.cancel(active);
        status("Interrupted");
      }
      FollowPath.clearPendingEventTriggers();
      active = null;
    }
    previouslyEnabled = ownsDrive;
    if (ownsDrive && (active == null || !scheduler.isScheduledOrRunning(active))) {
      drive.neutral();
    }
  }

  private void start() {
    fault.set(false);
    var routine = chooser.get();
    if (routine == null) {
      routine = DO_NOTHING;
    }
    Logger.recordOutput("Auto/Selected", routine.name());
    Logger.recordOutput("Auto/Segment", "");
    if (routine == DO_NOTHING) {
      status("Idle");
      return;
    }
    active = run(routine);
    if (!scheduler.schedule(active).successful()) {
      fail("Could not schedule " + routine.name());
      active = null;
    }
  }

  Command run(AutoRoutine routine) {
    var context = new AutoContext[1];
    var finishedSteps = new boolean[1];
    var steps =
        Command.noRequirements(
                co -> {
                  var auto =
                      new AutoContext(
                          co,
                          follower,
                          autosDirectory,
                          () -> pathFinished,
                          () -> pathFinished = false,
                          drive::setAutoStartPose,
                          () -> inputs.redAlliance);
                  context[0] = auto;
                  try {
                    routine.body().run(auto);
                  } catch (RuntimeException error) {
                    auto.fail(error.getClass().getSimpleName() + ": " + error.getMessage());
                  }
                  finishedSteps[0] = true;
                })
            .named(routine.name() + " Steps");

    return Command.noRequirements(
            co -> {
              double startTime = Timer.getTimestamp();
              status("Running");
              co.await(steps.withTimeout(Seconds.of(routine.timeoutSeconds())));
              Logger.recordOutput("Auto/FinalPose", drive.getPose());
              Logger.recordOutput("Auto/ElapsedSeconds", Timer.getTimestamp() - startTime);
              String failure = context[0] == null ? null : context[0].failure();
              if (failure != null) {
                fail(failure);
              } else if (!finishedSteps[0]) {
                fail("Timed out after " + routine.timeoutSeconds() + " s: " + routine.name());
              } else {
                status("Completed");
              }
            })
        .whenExited(
            () -> {
              drive.neutral();
              // Leaves the requested position alone so the joints hold where the routine ended.
              robot.requestClaw(Source.AUTO, Claw.Request.STOP);
            })
        .named(routine.name());
  }

  private void status(String value) {
    lastStatus = value;
    Logger.recordOutput("Auto/Status", value);
  }

  String lastStatus() {
    return lastStatus;
  }

  /** Routines in chooser order, after Do Nothing. */
  List<AutoRoutine> routines() {
    return routines;
  }

  private void fail(String message) {
    status(message);
    fault.setText(message);
    fault.set(true);
    drive.neutral();
  }
}

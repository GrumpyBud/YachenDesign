package first.robot;

import static org.wpilib.units.Units.Seconds;

import bline.commands.FollowPath;
import bline.field.FlippingUtil;
import bline.following.DriveType;
import bline.path.Path;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.vision.VisionConstants;
import first.robot.util.LoggedTunableNumber;
import java.io.File;
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

final class AutoRoutines {
  private record Routine(String name, String pathFile, double timeoutSeconds) {
    Path loadPath(File autosDirectory) {
      var path = new Path(autosDirectory, pathFile);
      if (!path.isValid() || path.getAuthoredStartPose().isEmpty()) {
        throw new IllegalArgumentException("Invalid path or missing authored start");
      }
      return path;
    }
  }

  private static final Routine DO_NOTHING = new Routine("Do Nothing", "", 0);
  private static final Routine[] ROUTINES = {
    new Routine("Drive 1 Meter", "drive-1m", 5),
    new Routine("Strafe 1 Meter (Calibration)", "strafe-1m", 5),
    new Routine("Turn 90 Degrees (Calibration)", "turn-90", 5)
  };

  /** Gradle entry point for checking deploy assets without starting a robot. */
  public static void main(String[] args) {
    var autosDirectory = new File(args[0]);
    boolean valid = true;
    for (var routine : ROUTINES) {
      try {
        routine.loadPath(autosDirectory);
        System.out.println("Validated " + routine.name() + " (" + routine.pathFile() + ".json)");
      } catch (RuntimeException error) {
        valid = false;
        System.err.println(
            routine.name() + " (" + routine.pathFile() + ".json): " + error.getMessage());
      }
    }
    if (!valid) {
      throw new IllegalStateException("Fix the BLine paths listed above before continuing.");
    }
  }

  @AutoLog
  static class AutoInputs {
    public boolean enabled;
    public boolean redAlliance;
  }

  private final Drive drive;
  private final LoggedNetworkChooser<Routine> chooser = new LoggedNetworkChooser<>("Auto Routine");
  private final Scheduler scheduler = Scheduler.getDefault();
  private final AutoInputsAutoLogged inputs = new AutoInputsAutoLogged();
  private final LoggedTunableNumber translationKp =
      new LoggedTunableNumber("BLine/TranslationKp", 2);
  private final LoggedTunableNumber rotationKp = new LoggedTunableNumber("BLine/RotationKp", 3);
  private final LoggedTunableNumber crossTrackKp =
      new LoggedTunableNumber("BLine/CrossTrackKp", 1.5);
  private final PIDController translationController = new PIDController(0, 0, 0);
  private final PIDController rotationController = new PIDController(0, 0, 0);
  private final PIDController crossTrackController = new PIDController(0, 0, 0);
  private final FollowPath.Builder follower;
  private final Alert fault = new Alert("AutonomousFault", "Autonomous stopped.", Level.HIGH);
  private Command active;
  private boolean previouslyEnabled;
  private boolean pathFinished;

  AutoRoutines(Drive drive) {
    this.drive = drive;
    chooser.addDefault(DO_NOTHING.name(), DO_NOTHING);
    for (var routine : ROUTINES) {
      chooser.add(routine.name(), routine);
    }

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

  boolean updateInputs() {
    if (Constants.currentMode != Constants.Mode.REPLAY) {
      inputs.enabled = RobotState.isAutonomousEnabled();
      inputs.redAlliance = MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
    }
    Logger.processInputs("Auto/Mode", inputs);
    return inputs.enabled;
  }

  void periodic(boolean ownsDrive) {
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
        Logger.recordOutput("Auto/Status", "Interrupted");
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
    Logger.recordOutput("Auto/Path", routine.pathFile());
    if (routine == DO_NOTHING) {
      Logger.recordOutput("Auto/Status", "Idle");
      return;
    }

    try {
      var path = routine.loadPath(new File(Filesystem.getDeployDirectory(), "autos"));
      active = follow(routine, path);
      if (!scheduler.schedule(active).successful()) {
        fail("Could not schedule " + routine.name());
        active = null;
      }
    } catch (RuntimeException error) {
      fail("Could not load " + routine.pathFile() + ": " + error.getMessage());
    }
  }

  private Command follow(Routine routine, Path path) {
    var segment =
        follower.build(path).withPoseReset().withTimeout(Seconds.of(routine.timeoutSeconds()));
    return Command.noRequirements(
            co -> {
              pathFinished = false;
              double startTime = Timer.getTimestamp();
              Logger.recordOutput("Auto/Status", "Running");
              var result = co.await(segment);
              Logger.recordOutput("Auto/FinalPose", drive.getPose());
              Logger.recordOutput("Auto/ElapsedSeconds", Timer.getTimestamp() - startTime);
              if (result.successful() && pathFinished) {
                Logger.recordOutput("Auto/Status", "Completed");
              } else {
                fail("Path did not reach its endpoint: " + routine.pathFile());
              }
            })
        .whenExited(drive::neutral)
        .named(routine.name());
  }

  private void fail(String message) {
    Logger.recordOutput("Auto/Status", message);
    fault.setText(message);
    fault.set(true);
    drive.neutral();
  }
}

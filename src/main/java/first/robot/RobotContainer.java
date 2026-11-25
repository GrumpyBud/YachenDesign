package first.robot;

import first.robot.auto.AutoRoutines;
import first.robot.generated.TunerConstants;
import first.robot.sim.SimGamePieces;
import first.robot.subsystems.Choreographer;
import first.robot.subsystems.Choreographer.State;
import first.robot.subsystems.arm.Arm;
import first.robot.subsystems.arm.ArmIO;
import first.robot.subsystems.arm.ArmIOSim;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.claw.ClawIO;
import first.robot.subsystems.claw.ClawIOSim;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.ModuleIO;
import first.robot.subsystems.drive.ModuleIOSim;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.elevator.ElevatorIO;
import first.robot.subsystems.elevator.ElevatorIOSim;
import first.robot.subsystems.vision.Vision;
import first.robot.subsystems.vision.VisionConstants;
import first.robot.subsystems.vision.VisionIO;
import first.robot.subsystems.vision.VisionIOPhotonVision;
import first.robot.subsystems.vision.VisionIOPhotonVisionSim;
import first.robot.util.FullSubsystem;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/** Wires up subsystems, their IO, driver controls, and autonomous routines. */
public final class RobotContainer {
  private final Drive drive;
  private final Controls controls = new Controls();
  private final AutoRoutines autos;
  private final Vision vision;
  private final Choreographer choreographer;
  private final List<FullSubsystem> subsystems;
  private final Alert configurationAlert =
      new Alert(
          "SwerveConfig",
          "Swerve hardware disabled. Verify the Tuner constants before enabling.",
          Level.HIGH);

  public RobotContainer() {
    drive =
        switch (Constants.currentMode) {
          case REAL ->
              Constants.ENABLE_REAL_DRIVE
                  ? new Drive(
                      new GyroIOPigeon2(),
                      new ModuleIOTalonFX(TunerConstants.FrontLeft),
                      new ModuleIOTalonFX(TunerConstants.FrontRight),
                      new ModuleIOTalonFX(TunerConstants.BackLeft),
                      new ModuleIOTalonFX(TunerConstants.BackRight))
                  : null;
          case SIM ->
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(TunerConstants.FrontLeft),
                  new ModuleIOSim(TunerConstants.FrontRight),
                  new ModuleIOSim(TunerConstants.BackLeft),
                  new ModuleIOSim(TunerConstants.BackRight));
          case REPLAY ->
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
        };
    configurationAlert.set(drive == null);
    Logger.recordOutput("Drive/Configured", drive != null);
    vision = drive == null ? null : createVision(drive);
    if (drive == null) {
      choreographer = null;
      autos = null;
      subsystems = List.of();
      return;
    }
    // Only simulation IO exists so far; real mode gets disconnected placeholders.
    boolean sim = Constants.currentMode == Constants.Mode.SIM;
    var elevator = new Elevator(sim ? new ElevatorIOSim() : new ElevatorIO() {});
    var arm = new Arm(sim ? new ArmIOSim() : new ArmIO() {});
    var claw =
        new Claw(
            sim
                ? new ClawIOSim(
                    new SimGamePieces(
                        drive::getSimPose, elevator::getPositionMeters, arm::getPositionRad))
                : new ClawIO() {});
    choreographer = new Choreographer(drive, elevator, arm, claw);
    autos = new AutoRoutines(drive, choreographer);
    subsystems = List.of(drive, elevator, arm, claw);
  }

  private static Vision createVision(Drive drive) {
    VisionIO[] cameras =
        switch (Constants.currentMode) {
          case REAL ->
              VisionConstants.REAL_CAMERAS.stream()
                  .map(camera -> new VisionIOPhotonVision(camera.name(), camera.robotToCamera()))
                  .toArray(VisionIO[]::new);
          case SIM -> {
            var camera = VisionConstants.SIM_CAMERA;
            yield new VisionIO[] {
              new VisionIOPhotonVisionSim(camera.name(), camera.robotToCamera(), drive::getSimPose)
            };
          }
          case REPLAY -> {
            var replayCameras = new VisionIO[Math.max(1, VisionConstants.REAL_CAMERAS.size())];
            Arrays.setAll(replayCameras, index -> new VisionIO() {});
            yield replayCameras;
          }
        };
    return new Vision(drive::addVisionMeasurement, cameras);
  }

  void periodic() {
    subsystems.forEach(FullSubsystem::periodic);
    if (drive != null) {
      vision.periodic();
      controls.updateInputs();
      State state =
          choreographer.periodic(
              autos.updateInputs(),
              controls.teleopEnabled(),
              controls.utilityEnabled(),
              controls.connected());
      autos.periodic(state == State.AUTONOMOUS);
      controls.apply(state, choreographer);
    }
  }

  void periodicAfterScheduler() {
    if (choreographer != null) {
      choreographer.applyOutputs();
    }
    subsystems.forEach(FullSubsystem::periodicAfterScheduler);
  }
}

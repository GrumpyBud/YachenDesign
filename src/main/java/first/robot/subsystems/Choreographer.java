package first.robot.subsystems;

import first.robot.subsystems.arm.Arm;
import first.robot.subsystems.arm.ArmConstants;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveCharacterization;
import first.robot.subsystems.drive.TeleopDrive;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.elevator.ElevatorConstants;
import first.robot.util.LoggedTunableNumber;
import java.util.EnumMap;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * The robot's state machine. It resolves the mode into who owns each output, applies the driver's
 * requests while the driver has control, coordinates the elevator and arm through named positions,
 * and enforces the stop interlocks. Subsystems only do what they are told here or by autonomous
 * commands. The elevator and arm move at the same time; no collision sequencing is known yet.
 */
public final class Choreographer {
  private enum Goal {
    IDLE,
    DRIVER_CONTROL,
    AUTONOMOUS,
    CHARACTERIZATION
  }

  public enum State {
    IDLE,
    WAITING_FOR_CONTROLLER,
    DRIVER_CONTROL,
    AUTONOMOUS,
    CHARACTERIZATION
  }

  /** Elevator and arm positions taken from the three CAD exports. */
  public enum Position {
    STOW(0.0, 42.0),
    INTAKE(0.224, -19.1),
    SCORE(1.308, 16.5);

    public final double defaultElevatorMeters;
    public final double defaultArmDegrees;

    Position(double elevatorMeters, double armDegrees) {
      defaultElevatorMeters = elevatorMeters;
      defaultArmDegrees = armDegrees;
    }
  }

  /** Who is asking for a mechanism action. Only the owner of the current state is obeyed. */
  public enum Source {
    DRIVER,
    AUTO
  }

  /**
   * The driver's drivetrain and utility-mode inputs for this loop. Controls handles button edges,
   * so {@code setDriverForward} is one-shot. Mechanisms are requested separately with a {@link
   * Source}.
   */
  public record DriverRequest(
      double forward,
      double left,
      double rotation,
      boolean xLock,
      boolean setDriverForward,
      boolean wheelRadiusTest,
      boolean feedforwardTest) {}

  private static final DriverRequest NO_REQUEST =
      new DriverRequest(0, 0, 0, false, false, false, false);

  private record Setpoint(LoggedTunableNumber elevatorMeters, LoggedTunableNumber armDegrees) {}

  private static final Translation3d ELEVATOR_AXIS =
      new Translation3d(
          Math.cos(ElevatorConstants.TILT_RAD), 0.0, Math.sin(ElevatorConstants.TILT_RAD));

  private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final Claw claw;
  private final TeleopDrive teleopDrive;
  private final DriveCharacterization characterization;
  private final EnumMap<Position, Setpoint> setpoints = new EnumMap<>(Position.class);
  private final Alert controllerAlert =
      new Alert("DriverController", "Driver controller on port 0 is disconnected.", Level.MEDIUM);
  private State state = State.IDLE;
  private Position position = Position.STOW;
  private DriverRequest request = NO_REQUEST;
  private Claw.Request clawRequest = Claw.Request.STOP;
  private Source lastOwner = null;

  public Choreographer(Drive drive, Elevator elevator, Arm arm, Claw claw) {
    this.drive = drive;
    this.elevator = elevator;
    this.arm = arm;
    this.claw = claw;
    teleopDrive = new TeleopDrive(drive);
    characterization = new DriveCharacterization(drive);
    for (var position : Position.values()) {
      String key = "Choreographer/" + position.name() + "/";
      setpoints.put(
          position,
          new Setpoint(
              new LoggedTunableNumber(key + "ElevatorMeters", position.defaultElevatorMeters),
              new LoggedTunableNumber(key + "ArmDegrees", position.defaultArmDegrees)));
    }
  }

  /** Resolve ownership from inputs that callers have already recorded for replay. */
  public State periodic(
      boolean autoEnabled,
      boolean teleopEnabled,
      boolean utilityEnabled,
      boolean controllerConnected) {
    Goal goal =
        autoEnabled
            ? Goal.AUTONOMOUS
            : teleopEnabled
                ? Goal.DRIVER_CONTROL
                : utilityEnabled ? Goal.CHARACTERIZATION : Goal.IDLE;
    state =
        switch (goal) {
          case IDLE -> State.IDLE;
          case AUTONOMOUS -> State.AUTONOMOUS;
          case DRIVER_CONTROL ->
              controllerConnected ? State.DRIVER_CONTROL : State.WAITING_FOR_CONTROLLER;
          case CHARACTERIZATION ->
              controllerConnected ? State.CHARACTERIZATION : State.WAITING_FOR_CONTROLLER;
        };
    controllerAlert.set(state == State.WAITING_FOR_CONTROLLER);
    if (state == State.AUTONOMOUS) {
      teleopDrive.resetDriverForward();
    }
    // A claw command never outlives its owner: a new owner, or no owner, starts stopped.
    var owner = owner();
    if (owner != lastOwner) {
      clawRequest = Claw.Request.STOP;
      lastOwner = owner;
    }
    Logger.recordOutput("Choreographer/Goal", goal);
    Logger.recordOutput("Choreographer/State", state);
    return state;
  }

  /** Set this loop's driver request. It is applied after the scheduler, then cleared. */
  public void setDriverRequest(DriverRequest request) {
    this.request = request;
  }

  /** The source that owns the mechanisms in the current state, or null when nobody does. */
  private Source owner() {
    return switch (state) {
      case DRIVER_CONTROL -> Source.DRIVER;
      case AUTONOMOUS -> Source.AUTO;
      case IDLE, WAITING_FOR_CONTROLLER, CHARACTERIZATION -> null;
    };
  }

  /**
   * Request an elevator and arm position. The position stays requested, including across a change
   * of owner, until someone requests another.
   *
   * @return false, and nothing changes, if {@code source} does not own the mechanisms right now
   */
  public boolean requestPosition(Source source, Position position) {
    if (source != owner()) {
      return false;
    }
    this.position = position;
    return true;
  }

  /**
   * Request claw rollers. The request stays active until changed or until ownership changes, so a
   * driver sends it every loop while a trigger is held, and a routine can leave the intake running
   * while it drives.
   *
   * @return false, and nothing changes, if {@code source} does not own the mechanisms right now
   */
  public boolean requestClaw(Source source, Claw.Request request) {
    if (source != owner()) {
      return false;
    }
    clawRequest = request;
    return true;
  }

  /** Whether the claw has detected a game piece from motor current. */
  public boolean hasGamePiece() {
    return claw.hasGamePiece();
  }

  public Claw.Request getClawRequest() {
    return clawRequest;
  }

  public Position getPosition() {
    return position;
  }

  /**
   * Whether both joints are within tolerance of the requested position. This compares against the
   * position's setpoints directly, so it is false right after a new request even before the joints
   * receive their new goals.
   */
  public boolean atPosition() {
    var setpoint = setpoints.get(position);
    return elevator.isNear(setpoint.elevatorMeters().get())
        && arm.isNear(Math.toRadians(setpoint.armDegrees().get()));
  }

  /**
   * Runs after the scheduler and before subsystems apply outputs. Applies the driver request for
   * the current state, pushes the position goals, and enforces the stop interlocks.
   */
  public void applyOutputs() {
    switch (state) {
      case DRIVER_CONTROL -> {
        if (request.setDriverForward()) {
          teleopDrive.setDriverForward();
        }
        if (request.xLock()) {
          drive.stopWithX();
        } else {
          teleopDrive.run(request.forward(), request.left(), request.rotation());
        }
      }
      case IDLE, WAITING_FOR_CONTROLLER -> drive.neutral();
      case AUTONOMOUS, CHARACTERIZATION -> {}
    }
    if (owner() != null) {
      claw.request(clawRequest);
    }
    // Called every loop so a running test stops cleanly when the state changes.
    characterization.periodic(
        state == State.CHARACTERIZATION, request.wheelRadiusTest(), request.feedforwardTest());
    request = NO_REQUEST;

    var setpoint = setpoints.get(position);
    elevator.setGoal(setpoint.elevatorMeters().get());
    arm.setGoal(Math.toRadians(setpoint.armDegrees().get()));
    // Joints keep their goals while enabled (including without a controller) so the elevator and
    // arm hold position, and resume those goals from the measured state after a disable.
    if (state == State.IDLE) {
      elevator.neutral();
      arm.neutral();
    }

    Logger.recordOutput("Choreographer/Position", position);
    Logger.recordOutput("Choreographer/AtPosition", atPosition());
    Logger.recordOutput("Choreographer/ClawRequest", clawRequest);
    Logger.recordOutput("Choreographer/Owner", owner() == null ? "None" : owner().name());
    Logger.recordOutput(
        "Choreographer/ComponentPoses",
        componentPoses(elevator.getPositionMeters(), arm.getPositionRad()));
    Logger.recordOutput(
        "Choreographer/GoalComponentPoses",
        componentPoses(elevator.getGoalMeters(), arm.getGoalRad()));
  }

  /** Poses for the AdvantageScope models: [0] first stage, [1] carriage, [2] arm and claw. */
  public static Pose3d[] componentPoses(double elevatorMeters, double armRad) {
    double stageMeters =
        Math.max(0.0, elevatorMeters - ElevatorConstants.CARRIAGE_INNER_TRAVEL_METERS);
    var carriage = ELEVATOR_AXIS.times(elevatorMeters);
    // Models are exported at the stowed pose, so each pose is the motion away from stow.
    var armRotation = new Rotation3d(0.0, -(armRad - ArmConstants.STOW_ANGLE_RAD), 0.0);
    var pivot = ArmConstants.PIVOT_AT_ZERO_TRAVEL;
    return new Pose3d[] {
      new Pose3d(ELEVATOR_AXIS.times(stageMeters), new Rotation3d()),
      new Pose3d(carriage, new Rotation3d()),
      new Pose3d(carriage.plus(pivot).minus(pivot.rotateBy(armRotation)), armRotation)
    };
  }
}

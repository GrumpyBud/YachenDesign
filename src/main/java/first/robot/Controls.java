package first.robot;

import first.robot.subsystems.Choreographer;
import first.robot.subsystems.Choreographer.DriverRequest;
import first.robot.subsystems.Choreographer.Position;
import first.robot.subsystems.Choreographer.Source;
import first.robot.subsystems.Choreographer.State;
import first.robot.subsystems.claw.Claw;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.driverstation.RobotState;

/**
 * Reads gamepad 0 and the robot mode, records them for replay, and turns them into one driver
 * request per loop. Button meaning lives here; deciding what runs belongs to the Choreographer.
 *
 * <p>Momentary buttons need a fresh press after each enable or change of control, so a button held
 * through a disable does nothing until it is released.
 */
final class Controls {
  private static final double TRIGGER_THRESHOLD = 0.5;
  private final Gamepad controller = new Gamepad(0);
  private final ControllerInputsAutoLogged inputs = new ControllerInputsAutoLogged();
  private boolean xLockReady;
  private boolean driverForwardReady;
  private boolean positionReady;
  private boolean clawReady;

  @AutoLog
  static class ControllerInputs {
    public boolean teleopEnabled;
    public boolean utilityEnabled;
    public boolean connected;
    // Drive
    public double forward;
    public double left;
    public double rotation;
    public boolean xLock;
    public boolean driverForward;
    // Mechanisms
    public boolean stow;
    public boolean intakePosition;
    public boolean scorePosition;
    public boolean clawIntake;
    public boolean clawEject;
    // Utility mode
    public boolean wheelRadiusTest;
    public boolean feedforwardTest;
  }

  Controls() {
    controller.setLeftXDeadband(0);
    controller.setLeftYDeadband(0);
    controller.setRightXDeadband(0);
  }

  void updateInputs() {
    if (Constants.currentMode != Constants.Mode.REPLAY) {
      inputs.teleopEnabled = RobotState.isTeleopEnabled();
      inputs.utilityEnabled = RobotState.isUtilityEnabled();
      inputs.connected = controller.isConnected();
      boolean c = inputs.connected;
      inputs.forward = c ? -controller.getLeftY() : 0;
      inputs.left = c ? -controller.getLeftX() : 0;
      inputs.rotation = c ? -controller.getRightX() : 0;
      inputs.xLock = c && controller.getFaceLeftButton();
      inputs.driverForward = c && controller.getBackButton();
      inputs.stow = c && controller.getFaceDownButton();
      inputs.intakePosition = c && controller.getFaceRightButton();
      inputs.scorePosition = c && controller.getFaceUpButton();
      inputs.clawIntake = c && controller.getRightTrigger() > TRIGGER_THRESHOLD;
      inputs.clawEject = c && controller.getLeftTrigger() > TRIGGER_THRESHOLD;
      inputs.wheelRadiusTest = c && controller.getLeftBumperButton();
      inputs.feedforwardTest = c && controller.getRightBumperButton();
    }
    Logger.processInputs("Controls", inputs);
  }

  boolean teleopEnabled() {
    return inputs.teleopEnabled;
  }

  boolean utilityEnabled() {
    return inputs.utilityEnabled;
  }

  boolean connected() {
    return inputs.connected;
  }

  /**
   * Sends this loop's driver request to the Choreographer. Mechanism buttons go through the same
   * source-checked requests as autonomous, so they only act while the driver has control.
   */
  void apply(State state, Choreographer robot) {
    if (state != State.DRIVER_CONTROL) {
      xLockReady = false;
      driverForwardReady = false;
      positionReady = false;
      clawReady = false;
      robot.setDriverRequest(
          new DriverRequest(0, 0, 0, false, false, inputs.wheelRadiusTest, inputs.feedforwardTest));
      return;
    }

    // X-lock holds while the button stays down after a fresh press.
    if (!inputs.xLock) {
      xLockReady = true;
    }
    boolean xLock = inputs.xLock && xLockReady;

    boolean setDriverForward = false;
    if (!inputs.driverForward) {
      driverForwardReady = true;
    } else if (driverForwardReady) {
      driverForwardReady = false;
      setDriverForward = true;
    }

    if (!inputs.stow && !inputs.intakePosition && !inputs.scorePosition) {
      positionReady = true;
    } else if (positionReady) {
      positionReady = false;
      robot.requestPosition(
          Source.DRIVER,
          inputs.scorePosition
              ? Position.SCORE
              : inputs.intakePosition ? Position.INTAKE : Position.STOW);
    }

    var claw = Claw.Request.STOP;
    if (!inputs.clawIntake && !inputs.clawEject) {
      clawReady = true;
    } else if (clawReady) {
      claw = inputs.clawEject ? Claw.Request.EJECT : Claw.Request.INTAKE;
    }
    // Sent every loop, so releasing the trigger stops the rollers.
    robot.requestClaw(Source.DRIVER, claw);

    robot.setDriverRequest(
        new DriverRequest(
            inputs.forward,
            inputs.left,
            inputs.rotation,
            xLock,
            setDriverForward,
            inputs.wheelRadiusTest,
            inputs.feedforwardTest));
  }
}

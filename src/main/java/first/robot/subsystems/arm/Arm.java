package first.robot.subsystems.arm;

import static first.robot.subsystems.arm.ArmConstants.*;

import first.robot.util.FullSubsystem;
import first.robot.util.GoalTimeout;
import first.robot.util.LoggedTunableNumber;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/** Holds the arm angle goal and applies it after commands and interlocks run. */
public class Arm extends FullSubsystem {
  private static final double HOT_MOTOR_CELSIUS = 70.0;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final Alert disconnectedAlert =
      new Alert("ArmDisconnected", "Arm motor disconnected.", Level.HIGH);
  private final Alert hotAlert = new Alert("ArmHot", "Arm motor above 70 C.", Level.MEDIUM);
  private final GoalTimeout goalTimeout =
      new GoalTimeout("ArmGoalTimeout", "Arm has not reached its goal. Holding goal.");

  private final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 30.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 1.0);
  private final LoggedTunableNumber cruise = new LoggedTunableNumber("Arm/CruiseRadPerSec", 6.0);
  private final LoggedTunableNumber acceleration =
      new LoggedTunableNumber("Arm/AccelerationRadPerSecSq", 20.0);
  private final LoggedTunableNumber timeoutSeconds =
      new LoggedTunableNumber("Arm/GoalTimeoutSeconds", 2.0);
  private double[] appliedGains = new double[0];

  private double goalRad = STOW_ANGLE_RAD;
  private boolean neutralRequested = false;

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    disconnectedAlert.set(!inputs.connected);
    hotAlert.set(inputs.tempCelsius > HOT_MOTOR_CELSIUS);
  }

  /** Set the angle goal, clamped to the provisional range. The goal survives disable. */
  public void setGoal(double positionRad) {
    goalRad = Math.clamp(positionRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
  }

  /** Stop driving the motor for this loop. */
  public void neutral() {
    neutralRequested = true;
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }

  public double getGoalRad() {
    return goalRad;
  }

  public boolean atGoal() {
    return isNear(goalRad);
  }

  /** Whether the measured angle is within tolerance of an angle, with a connected motor. */
  public boolean isNear(double positionRad) {
    return inputs.connected && Math.abs(inputs.positionRad - positionRad) < AT_GOAL_TOLERANCE_RAD;
  }

  @Override
  public void periodicAfterScheduler() {
    double[] gains = {kP.get(), kD.get(), cruise.get(), acceleration.get()};
    if (!Arrays.equals(gains, appliedGains)) {
      io.configure(gains[0], gains[1], gains[2], gains[3]);
      appliedGains = gains;
    }
    if (neutralRequested) {
      io.neutral();
    } else {
      io.runPosition(goalRad);
    }
    boolean atGoal = atGoal();
    boolean timedOut = goalTimeout.update(!neutralRequested, goalRad, atGoal, timeoutSeconds.get());

    Logger.recordOutput("Arm/GoalDegrees", Math.toDegrees(goalRad));
    Logger.recordOutput("Arm/PositionDegrees", Math.toDegrees(inputs.positionRad));
    Logger.recordOutput("Arm/Neutral", neutralRequested);
    Logger.recordOutput("Arm/AtGoal", atGoal);
    Logger.recordOutput(
        "Diagnostics/Arm/GoalErrorDegrees", Math.toDegrees(goalRad - inputs.positionRad));
    Logger.recordOutput(
        "Diagnostics/Arm/TrackingErrorDegrees",
        Math.toDegrees(inputs.setpointRad - inputs.positionRad));
    Logger.recordOutput("Diagnostics/Arm/SecondsOnGoal", goalTimeout.elapsedSeconds());
    Logger.recordOutput("Diagnostics/Arm/GoalTimedOut", timedOut);
    Logger.recordOutput("Diagnostics/Arm/StatorCurrentAmps", inputs.statorCurrentAmps);
    neutralRequested = false;
  }
}

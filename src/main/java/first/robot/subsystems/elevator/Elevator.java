package first.robot.subsystems.elevator;

import static first.robot.subsystems.elevator.ElevatorConstants.*;

import first.robot.util.FullSubsystem;
import first.robot.util.GoalTimeout;
import first.robot.util.LoggedTunableNumber;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/** Holds the elevator travel goal and applies it after commands and interlocks run. */
public class Elevator extends FullSubsystem {
  private static final double HOT_MOTOR_CELSIUS = 70.0;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert leaderDisconnectedAlert =
      new Alert("ElevatorLeader", "Elevator leader motor disconnected.", Level.HIGH);
  private final Alert followerDisconnectedAlert =
      new Alert("ElevatorFollower", "Elevator follower motor disconnected.", Level.HIGH);
  private final Alert hotAlert =
      new Alert("ElevatorHot", "Elevator motor above 70 C.", Level.MEDIUM);
  private final GoalTimeout goalTimeout =
      new GoalTimeout("ElevatorGoalTimeout", "Elevator has not reached its goal. Holding goal.");

  private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 40.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 1.0);
  private final LoggedTunableNumber cruise =
      new LoggedTunableNumber("Elevator/CruiseMetersPerSec", 2.5);
  private final LoggedTunableNumber acceleration =
      new LoggedTunableNumber("Elevator/AccelerationMetersPerSecSq", 8.0);
  private final LoggedTunableNumber timeoutSeconds =
      new LoggedTunableNumber("Elevator/GoalTimeoutSeconds", 3.0);
  private double[] appliedGains = new double[0];

  private double goalMeters = MIN_TRAVEL_METERS;
  private boolean neutralRequested = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    leaderDisconnectedAlert.set(!inputs.leaderConnected);
    followerDisconnectedAlert.set(!inputs.followerConnected);
    hotAlert.set(Arrays.stream(inputs.tempCelsius).anyMatch(t -> t > HOT_MOTOR_CELSIUS));
  }

  /** Set the travel goal, clamped to the provisional travel range. The goal survives disable. */
  public void setGoal(double positionMeters) {
    goalMeters = Math.clamp(positionMeters, MIN_TRAVEL_METERS, MAX_TRAVEL_METERS);
  }

  /** Stop driving the motors for this loop. */
  public void neutral() {
    neutralRequested = true;
  }

  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  public double getGoalMeters() {
    return goalMeters;
  }

  /** At goal only with a connected leader, whose encoder is the position source. */
  public boolean atGoal() {
    return inputs.leaderConnected
        && Math.abs(inputs.positionMeters - goalMeters) < AT_GOAL_TOLERANCE_METERS;
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
      io.runPosition(goalMeters);
    }
    boolean atGoal = atGoal();
    boolean timedOut =
        goalTimeout.update(!neutralRequested, goalMeters, atGoal, timeoutSeconds.get());

    Logger.recordOutput("Elevator/GoalMeters", goalMeters);
    Logger.recordOutput("Elevator/Neutral", neutralRequested);
    Logger.recordOutput("Elevator/AtGoal", atGoal);
    Logger.recordOutput("Diagnostics/Elevator/GoalErrorMeters", goalMeters - inputs.positionMeters);
    Logger.recordOutput(
        "Diagnostics/Elevator/TrackingErrorMeters", inputs.setpointMeters - inputs.positionMeters);
    Logger.recordOutput("Diagnostics/Elevator/SecondsOnGoal", goalTimeout.elapsedSeconds());
    Logger.recordOutput("Diagnostics/Elevator/GoalTimedOut", timedOut);
    Logger.recordOutput(
        "Diagnostics/Elevator/TotalStatorCurrentAmps",
        Arrays.stream(inputs.statorCurrentAmps).sum());
    neutralRequested = false;
  }
}

package first.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Elevator hardware boundary. Positions are carriage travel along the tilted elevator axis. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;

    /** The profiled reference the controller is tracking this loop (Motion Magic reference). */
    public double setpointMeters = 0.0;

    /** Per motor: [leader, follower]. */
    public double[] appliedVolts = new double[] {};

    public double[] statorCurrentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Profiled move to the travel goal (Motion Magic on Talon FX hardware). */
  default void runPosition(double positionMeters) {}

  /** Stop driving the motors. The next {@link #runPosition} starts from the measured state. */
  default void neutral() {}

  default void configure(
      double kP, double kD, double cruiseMetersPerSec, double accelerationMetersPerSecSq) {}
}

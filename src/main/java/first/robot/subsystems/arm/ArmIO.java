package first.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

/** Arm hardware boundary. Angles are arm elevation above the robot's forward horizontal. */
public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    /** The profiled reference the controller is tracking this loop (Motion Magic reference). */
    public double setpointRad = 0.0;

    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  /** Profiled move to the angle goal (Motion Magic on Talon FX hardware). */
  default void runPosition(double positionRad) {}

  /** Stop driving the motor. The next {@link #runPosition} starts from the measured state. */
  default void neutral() {}

  default void configure(
      double kP, double kD, double cruiseRadPerSec, double accelerationRadPerSecSq) {}
}

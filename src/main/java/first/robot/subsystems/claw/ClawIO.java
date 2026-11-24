package first.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

/** Roller claw hardware boundary. Velocity is roller-side. */
public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {
    public boolean connected = false;
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(ClawIOInputs inputs) {}

  default void setVoltage(double volts) {}
}

package first.robot.subsystems.claw;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

/** Free-spinning roller model with no game piece load. */
public class ClawIOSim implements ClawIO {
  // One Kraken X44 (user-confirmed; the motor is not in the CAD yet). Reduction and roller
  // inertia are estimates.
  private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(1);
  private static final double GEAR_RATIO = 3.0;
  private static final double MOI = 0.002;

  private final DCMotorSim sim =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(GEARBOX, MOI, GEAR_RATIO), GEARBOX);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);
    inputs.connected = true;
    inputs.velocityRotPerSec = sim.getAngularVelocity() / (2 * Math.PI);
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.abs(sim.getCurrentDraw());
    inputs.tempCelsius = 25.0; // Not modeled; ambient.
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = Math.clamp(volts, -12.0, 12.0);
  }
}

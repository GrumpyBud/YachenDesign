package first.robot.subsystems.claw;

import first.robot.sim.SimGamePieces;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

/**
 * Roller model. With a {@link SimGamePieces} field, a held piece jams the rollers while intaking:
 * the rollers stop and the motor draws stall current, limited like a Talon FX stator limit.
 */
public class ClawIOSim implements ClawIO {
  // One Kraken X44 (user-confirmed; the motor is not in the CAD yet). Reduction and roller
  // inertia are estimates.
  private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(1);
  private static final double GEAR_RATIO = 3.0;
  private static final double MOI = 0.002;
  private static final double STATOR_LIMIT_AMPS = 60.0;

  private final DCMotorSim sim =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(GEARBOX, MOI, GEAR_RATIO), GEARBOX);
  private final SimGamePieces pieces;
  private double appliedVolts = 0.0;

  /** Free-spinning rollers with no game pieces. */
  public ClawIOSim() {
    this(null);
  }

  public ClawIOSim(SimGamePieces pieces) {
    this.pieces = pieces;
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    boolean held = pieces != null && pieces.update(appliedVolts);
    boolean jammed = held && appliedVolts > 0.0;
    sim.setInputVoltage(jammed ? 0.0 : appliedVolts);
    sim.update(0.02);
    if (jammed) {
      sim.setAngularVelocity(0.0);
    }

    double motorRadPerSec = sim.getAngularVelocity() * GEAR_RATIO;
    double statorAmps = (appliedVolts - motorRadPerSec / GEARBOX.Kv) / GEARBOX.R;
    inputs.connected = true;
    inputs.velocityRotPerSec = sim.getAngularVelocity() / (2 * Math.PI);
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.min(Math.abs(statorAmps), STATOR_LIMIT_AMPS);
    inputs.tempCelsius = 25.0; // Not modeled; ambient.
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = Math.clamp(volts, -12.0, 12.0);
  }
}

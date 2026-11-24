package first.robot.subsystems.arm;

import static first.robot.subsystems.arm.ArmConstants.*;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.simulation.SingleJointedArmSim;

/**
 * Point-mass arm model with an onboard profile standing in for Motion Magic. SingleJointedArmSim
 * models a uniform rod, so a rod of 1.5x the pivot-to-mass distance gives the point mass's gravity.
 */
public class ArmIOSim implements ArmIO {
  private static final double DT = 0.02;
  private static final double MOI = MASS_KG * PIVOT_TO_CLAW_METERS * PIVOT_TO_CLAW_METERS;
  private static final double GRAVITY_VOLTS_HORIZONTAL =
      GEARBOX.getVoltage(MASS_KG * 9.81 * PIVOT_TO_CLAW_METERS / GEAR_RATIO, 0);
  private static final double KV = GEAR_RATIO / GEARBOX.Kv;
  private static final double KA = GEARBOX.getVoltage(MOI / GEAR_RATIO, 0);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          GEARBOX,
          GEAR_RATIO,
          MOI,
          1.5 * PIVOT_TO_CLAW_METERS,
          MIN_ANGLE_RAD,
          MAX_ANGLE_RAD,
          true,
          STOW_ANGLE_RAD);
  private final PIDController feedback = new PIDController(0, 0, 0);
  private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 1));
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private boolean closedLoop = false;
  private double goalRad = STOW_ANGLE_RAD;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (closedLoop) {
      var previous = setpoint;
      setpoint = profile.calculate(DT, setpoint, new TrapezoidProfile.State(goalRad, 0));
      double acceleration = (setpoint.velocity - previous.velocity) / DT;
      appliedVolts =
          GRAVITY_VOLTS_HORIZONTAL * Math.cos(setpoint.position)
              + KV * setpoint.velocity
              + KA * acceleration
              + feedback.calculate(sim.getAngle(), setpoint.position);
    } else {
      appliedVolts = 0.0;
    }
    appliedVolts = Math.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(DT);

    inputs.connected = true;
    inputs.positionRad = sim.getAngle();
    inputs.velocityRadPerSec = sim.getVelocity();
    inputs.setpointRad = closedLoop ? setpoint.position : sim.getAngle();
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.abs(sim.getCurrentDraw());
    inputs.tempCelsius = 25.0; // Not modeled; ambient.
  }

  @Override
  public void runPosition(double positionRad) {
    if (!closedLoop) {
      setpoint = new TrapezoidProfile.State(sim.getAngle(), sim.getVelocity());
      feedback.reset();
    }
    closedLoop = true;
    goalRad = positionRad;
  }

  @Override
  public void neutral() {
    closedLoop = false;
  }

  @Override
  public void configure(
      double kP, double kD, double cruiseRadPerSec, double accelerationRadPerSecSq) {
    feedback.setPID(kP, 0, kD);
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(cruiseRadPerSec, accelerationRadPerSecSq));
  }
}

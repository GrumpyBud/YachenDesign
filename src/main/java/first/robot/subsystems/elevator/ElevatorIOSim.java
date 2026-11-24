package first.robot.subsystems.elevator;

import static first.robot.subsystems.elevator.ElevatorConstants.*;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.simulation.ElevatorSim;

/**
 * Physics model of the tilted elevator with an onboard profile standing in for Motion Magic.
 * Gravity acts along the incline, g * sin(tilt), so it enters as an equivalent motor voltage
 * instead of through ElevatorSim's vertical-gravity option.
 */
public class ElevatorIOSim implements ElevatorIO {
  private static final double DT = 0.02;
  private static final double GRAVITY_VOLTS =
      GEARBOX.getVoltage(
          MOVING_MASS_KG * 9.81 * Math.sin(TILT_RAD) * DRUM_RADIUS_METERS / GEAR_RATIO, 0);
  // Ideal feedforward from the same model: volts per (m/s) and per (m/s^2).
  private static final double KV = GEAR_RATIO / (DRUM_RADIUS_METERS * GEARBOX.Kv);
  private static final double KA =
      GEARBOX.getVoltage(MOVING_MASS_KG * DRUM_RADIUS_METERS / GEAR_RATIO, 0);

  private final ElevatorSim sim =
      new ElevatorSim(
          GEARBOX,
          GEAR_RATIO,
          MOVING_MASS_KG,
          DRUM_RADIUS_METERS,
          MIN_TRAVEL_METERS,
          MAX_TRAVEL_METERS,
          false,
          MIN_TRAVEL_METERS);
  private final PIDController feedback = new PIDController(0, 0, 0);
  private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 1));
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private boolean closedLoop = false;
  private double goalMeters = 0.0;
  private double appliedVolts = 0.0;
  // Temperature is not modeled; report ambient so the field is populated.
  private static final double AMBIENT_CELSIUS = 25.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      var previous = setpoint;
      setpoint = profile.calculate(DT, setpoint, new TrapezoidProfile.State(goalMeters, 0));
      double acceleration = (setpoint.velocity - previous.velocity) / DT;
      appliedVolts =
          GRAVITY_VOLTS
              + KV * setpoint.velocity
              + KA * acceleration
              + feedback.calculate(sim.getPosition(), setpoint.position);
    } else {
      appliedVolts = 0.0;
    }
    appliedVolts = Math.clamp(appliedVolts, -12.0, 12.0);
    // Subtract the incline's gravity as an equivalent voltage on the motors.
    sim.setInputVoltage(appliedVolts - GRAVITY_VOLTS);
    sim.update(DT);

    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.positionMeters = sim.getPosition();
    inputs.velocityMetersPerSec = sim.getVelocity();
    inputs.setpointMeters = closedLoop ? setpoint.position : sim.getPosition();
    inputs.appliedVolts = new double[] {appliedVolts, appliedVolts};
    // ElevatorSim reports battery current for the gearbox; split it evenly between the motors.
    double current = Math.abs(sim.getCurrentDraw()) / 2.0;
    inputs.statorCurrentAmps = new double[] {current, current};
    inputs.tempCelsius = new double[] {AMBIENT_CELSIUS, AMBIENT_CELSIUS};
  }

  @Override
  public void runPosition(double positionMeters) {
    if (!closedLoop) {
      setpoint = new TrapezoidProfile.State(sim.getPosition(), sim.getVelocity());
      feedback.reset();
    }
    closedLoop = true;
    goalMeters = positionMeters;
  }

  @Override
  public void neutral() {
    closedLoop = false;
  }

  @Override
  public void configure(
      double kP, double kD, double cruiseMetersPerSec, double accelerationMetersPerSecSq) {
    feedback.setPID(kP, 0, kD);
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(cruiseMetersPerSec, accelerationMetersPerSecSq));
  }
}

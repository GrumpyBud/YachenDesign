package first.robot.subsystems.arm;

import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;

/**
 * Arm (claw pivot) facts. The angle is the elevation of the pivot-to-claw line above the robot's
 * forward horizontal, positive up. Sources are the 2026-11-24 Onshape exports (see
 * docs/robot/hardware/superstructure.md). Values marked "estimate" are not from CAD or measurement.
 */
public final class ArmConstants {
  private ArmConstants() {}

  /** One Kraken X60 (CAD). */
  public static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

  /** CAD: MAXPlanetary 5:1 and 3:1 stages. Provisional: no further reduction found after it. */
  public static final double GEAR_RATIO = 5.0 * 3.0;

  /** CAD: pivot axis in the robot frame with the elevator at zero travel. */
  public static final Translation3d PIVOT_AT_ZERO_TRAVEL = new Translation3d(-0.0975, 0.0, 0.2670);

  /** CAD: pivot to the claw's geometric center. */
  public static final double PIVOT_TO_CLAW_METERS = 0.541;

  /** CAD: arm angle in the stowed export (pivot-to-claw-center line). */
  public static final double STOW_ANGLE_RAD = Math.toRadians(42.0);

  /** Provisional software range; the real hard stops and collision limits are unknown. */
  public static final double MIN_ANGLE_RAD = Math.toRadians(-30.0);

  public static final double MAX_ANGLE_RAD = Math.toRadians(100.0);

  /** Estimate: arm and claw mass, treated as a point mass at the claw center. */
  public static final double MASS_KG = 3.0;

  public static final double AT_GOAL_TOLERANCE_RAD = Math.toRadians(2.0);
}

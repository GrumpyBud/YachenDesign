package first.robot.subsystems.elevator;

import org.wpilib.math.system.DCMotor;

/**
 * Elevator facts. Sources are the 2026-11-24 Onshape exports "Top Level Assembly Stowed / Intaking
 * (roughly) / Scoring" (see docs/robot/hardware/superstructure.md). Values marked "estimate" are
 * not from CAD or measurement.
 */
public final class ElevatorConstants {
  private ElevatorConstants() {}

  /** Two Kraken X60s on one gearbox (CAD). */
  public static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(2);

  /** CAD: 10t pinion driving a 38t gear. */
  public static final double GEAR_RATIO = 38.0 / 10.0;

  /** CAD: 24t HTD 5 mm output pulley. Assumes the carriage moves 1:1 with the belt. */
  public static final double DRUM_RADIUS_METERS = 24 * 0.005 / (2 * Math.PI);

  /** CAD: the carriage travels 39.95 degrees above horizontal, toward the robot front. */
  public static final double TILT_RAD = Math.atan2(0.8398, 1.0027);

  /** Zero is the CAD stowed pose. */
  public static final double MIN_TRAVEL_METERS = 0.0;

  /** Provisional. The CAD scoring pose is 1.308 m; the real hard stop is unknown. */
  public static final double MAX_TRAVEL_METERS = 1.32;

  /**
   * CAD: continuous rigging. The carriage climbs 0.609 m (24 in) inside the first stage before the
   * first stage starts moving. Used only for the 3D visualization.
   */
  public static final double CARRIAGE_INNER_TRAVEL_METERS = 0.6094;

  /** Estimate: carriage, first stage, pivot, and claw mass carried by the belt. */
  public static final double MOVING_MASS_KG = 8.0;

  public static final double AT_GOAL_TOLERANCE_METERS = 0.02;
}

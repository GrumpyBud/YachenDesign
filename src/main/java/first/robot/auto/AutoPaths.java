package first.robot.auto;

import bline.path.Path;
import bline.path.Path.PathConstraints;
import bline.path.Path.Waypoint;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Paths defined in Java. Coordinates are meters and are written from the blue alliance's side;
 * BLine flips them for red. Speeds, accelerations, and end tolerances default to {@code
 * deploy/autos/config.json}. Override them for one path with {@link PathConstraints}.
 *
 * <p>Add each path as a {@code public static Path name()} method with no parameters. The build's
 * {@code validateAutos} task finds and checks every such method automatically.
 *
 * <p>Element reference:
 *
 * <ul>
 *   <li>{@link Waypoint} - drive to a position and end at a heading. Start a routine's first path
 *       with one, because it sets the starting pose.
 *   <li>{@link TranslationTarget} - pass through a position without a heading goal.
 *   <li>{@link RotationTarget} - reach a heading partway (0 to 1) along the current segment.
 *   <li>{@link EventTrigger} - run a registered event partway along the current segment. See {@link
 *       Autos#registerEvents}.
 * </ul>
 */
public final class AutoPaths {
  private AutoPaths() {}

  /** Blue-side practice start shared by the starter paths. */
  private static final double START_X = 2.0;

  private static final double START_Y = 4.0;

  public static Path drive1Meter() {
    return new Path(
        new Waypoint(START_X, START_Y, Rotation2d.ZERO),
        new Waypoint(START_X + 1, START_Y, Rotation2d.ZERO));
  }

  public static Path strafe1Meter() {
    return new Path(
        new Waypoint(START_X, START_Y, Rotation2d.ZERO),
        new Waypoint(START_X, START_Y + 1, Rotation2d.ZERO));
  }

  public static Path turn90Degrees() {
    return new Path(
        new Waypoint(START_X, START_Y, Rotation2d.ZERO),
        new Waypoint(START_X, START_Y, Rotation2d.CCW_90DEG));
  }

  /** Return leg for the out-and-back example. Starts where {@link #drive1Meter()} ends. */
  public static Path driveBack1Meter() {
    return new Path(
        new PathConstraints().setMaxVelocityMetersPerSec(0.5),
        new Waypoint(START_X + 1, START_Y, Rotation2d.ZERO),
        new Waypoint(START_X, START_Y, Rotation2d.ZERO));
  }
}

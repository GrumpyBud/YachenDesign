package first.robot.auto;

import static first.robot.auto.ChargedUpField.*;

import bline.path.Path;
import bline.path.Path.EventTrigger;
import bline.path.Path.PathConstraints;
import bline.path.Path.PathElement;
import bline.path.Path.RotationTarget;
import bline.path.Path.TranslationTarget;
import bline.path.Path.Waypoint;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Builds the legs of a scoring cycle for any node and staged piece, on the blue side (BLine mirrors
 * them for red). Every trip between the grid and the pieces uses a lane above or below the charge
 * station, so the robot never crosses it.
 *
 * <p>The robot only turns east of the charge station, never in a lane: the lower lane has too
 * little room beside the wall for a turning robot.
 *
 * <p>The robot is tippy, so legs never carry an extended mechanism at speed. The routine retracts
 * fully before any leg starts. A leg only extends near its end, by distance remaining: the intake
 * lowers {@value #INTAKE_LOWER_REMAINING_METERS} m before stopping behind a piece, and the elevator
 * raises {@value #SCORE_RAISE_REMAINING_METERS} m before stopping at a node.
 */
public final class CyclePaths {
  private CyclePaths() {}

  static final double INTAKE_LOWER_REMAINING_METERS = 0.6;
  static final double SCORE_RAISE_REMAINING_METERS = 0.4;

  private static final Rotation2d FACING_GRID = Rotation2d.k180deg;
  private static final Rotation2d FACING_FIELD = Rotation2d.ZERO;

  // BLine commands speed = translation kP x remaining distance, then limits acceleration. To stop
  // at the end without overshoot, max velocity must stay below 2 x max acceleration / kP
  // (AutoRoutines' BLine/TranslationKp, 2.5 by default). Change these together. Do not set a
  // minimum velocity on a path that should stop: BLine then passes through the end instead.
  private static PathConstraints travel() {
    return new PathConstraints()
        .setMaxVelocityMetersPerSec(3.0) // Limit at kP 2.5: 2 x 4.5 / 2.5 = 3.6 m/s
        .setMaxAccelerationMetersPerSec2(4.5)
        // A 180 degree turn takes about 0.8 s at these limits; turns happen with everything stowed.
        .setMaxVelocityDegPerSec(540)
        // Same stop rule for turning: 540 deg/s <= 2 x 1440 / rotation kP (4) = 720 deg/s.
        .setMaxAccelerationDegPerSec2(1440);
  }

  /**
   * BLine slows only for the final stop, not for corners, so a sharp corner taken at speed
   * overshoots. Segments that end in a corner sharper than this are capped at {@link
   * #CORNER_SPEED_METERS_PER_SEC}; the robot speeds up again after the corner.
   */
  static final double SHARP_CORNER_DEGREES = 35;

  static final double CORNER_SPEED_METERS_PER_SEC = 1.5;

  /** Driving onto a piece with the claw down. */
  private static PathConstraints pickup() {
    return new PathConstraints()
        .setMaxVelocityMetersPerSec(1.8) // Limit at kP 2.5: 2 x 3.0 / 2.5 = 2.4 m/s
        .setMaxAccelerationMetersPerSec2(3.0);
  }

  /**
   * Grid to just behind a staged piece: into the lane, turn to face the field over the station,
   * then stop lined up behind the piece, lowering the intake with the rollers on just before
   * stopping.
   */
  public static Path gridToPiece(int node, int piece) {
    double nodeY = nodeY(node);
    double pieceY = stagedPieceY(piece);
    double lane = laneBetween(nodeY, pieceY);
    return build(
        travel(),
        List.of(
            new Translation2d(GRID_X, nodeY),
            new Translation2d(LANE_WEST_X, lane),
            new Translation2d(LANE_EAST_X, lane),
            new Translation2d(APPROACH_X, pieceY)),
        FACING_GRID,
        FACING_FIELD,
        // Hold the heading through the lane; turn only after leaving it, east of the station.
        List.of(new Turn(FACING_GRID, 1, 1.0)),
        Map.of(Autos.INTAKE, INTAKE_LOWER_REMAINING_METERS));
  }

  /** Drive straight onto a staged piece until the claw is just past its center. */
  public static Path ontoPiece(int piece) {
    return ontoPieceFrom(APPROACH_X, piece);
  }

  /** As {@link #ontoPiece}, from the closer stop used for a fallback piece. */
  public static Path ontoFallbackPiece(int piece) {
    return ontoPieceFrom(FALLBACK_APPROACH_X, piece);
  }

  private static Path ontoPieceFrom(double startX, int piece) {
    double y = stagedPieceY(piece);
    return build(
        pickup(),
        List.of(new Translation2d(startX, y), new Translation2d(PICKUP_END_X, y)),
        FACING_FIELD,
        FACING_FIELD,
        List.of(),
        Map.of());
  }

  /**
   * From a missed pickup to just behind another staged piece. Stays east of the charge station,
   * lowering the intake just before stopping.
   */
  public static Path pieceToPiece(int from, int to) {
    if (from == to) {
      throw new IllegalArgumentException("Fallback piece must differ from the missed piece");
    }
    return build(
        travel(),
        List.of(
            new Translation2d(PICKUP_END_X, stagedPieceY(from)),
            new Translation2d(FALLBACK_APPROACH_X, stagedPieceY(to))),
        FACING_FIELD,
        FACING_FIELD,
        List.of(),
        Map.of(Autos.INTAKE, INTAKE_LOWER_REMAINING_METERS));
  }

  /**
   * From a picked-up piece back through the lane, stopping at its west end. Turns to face the grid
   * before entering the lane, east of the station.
   */
  public static Path pieceToLane(int piece, int node) {
    double pieceY = stagedPieceY(piece);
    double lane = laneBetween(pieceY, nodeY(node));
    return build(
        travel(),
        List.of(
            new Translation2d(PICKUP_END_X, pieceY),
            new Translation2d(LANE_EAST_X, lane),
            new Translation2d(LANE_WEST_X, lane)),
        FACING_FIELD,
        FACING_GRID,
        List.of(new Turn(FACING_GRID, 0, 1.0)),
        Map.of());
  }

  /**
   * From the end of the lane straight into a node, raising to score just before stopping. Already
   * facing the grid, and west of the charge station the whole way.
   */
  public static Path laneToNode(int piece, int node) {
    double nodeY = nodeY(node);
    double lane = laneBetween(stagedPieceY(piece), nodeY);
    return build(
        travel(),
        List.of(new Translation2d(LANE_WEST_X, lane), new Translation2d(GRID_X, nodeY)),
        FACING_GRID,
        FACING_GRID,
        List.of(),
        Map.of(Autos.SCORE, SCORE_RAISE_REMAINING_METERS));
  }

  /** Authored length of a path through its translation points, in meters. */
  public static double length(Path path) {
    double total = 0;
    Translation2d previous = null;
    for (var element : path.getPathElements()) {
      Translation2d point =
          element instanceof Waypoint w
              ? w.translationTarget().translation()
              : element instanceof TranslationTarget t ? t.translation() : null;
      if (point != null) {
        if (previous != null) {
          total += previous.getDistance(point);
        }
        previous = point;
      }
    }
    return total;
  }

  /** Every leg any plan can produce, for build-time validation and clearance tests. */
  public static Map<String, Path> allLegs() {
    var legs = new LinkedHashMap<String, Path>();
    for (int piece = 1; piece <= STAGED_PIECE_COUNT; piece++) {
      legs.put("ontoPiece(" + piece + ")", ontoPiece(piece));
      legs.put("ontoFallbackPiece(" + piece + ")", ontoFallbackPiece(piece));
      for (int node = 1; node <= NODE_COUNT; node++) {
        legs.put("gridToPiece(" + node + ", " + piece + ")", gridToPiece(node, piece));
        legs.put("pieceToLane(" + piece + ", " + node + ")", pieceToLane(piece, node));
        legs.put("laneToNode(" + piece + ", " + node + ")", laneToNode(piece, node));
      }
      for (int to = 1; to <= STAGED_PIECE_COUNT; to++) {
        if (to != piece) {
          legs.put("pieceToPiece(" + piece + ", " + to + ")", pieceToPiece(piece, to));
        }
      }
    }
    return legs;
  }

  /** A heading target reached at {@code tRatio} along segment {@code segment} (0-based). */
  private record Turn(Rotation2d heading, int segment, double tRatio) {}

  /**
   * Assembles a path through {@code points}. Each event fires when the given distance remains to
   * the end along the path, so it lands at the same physical spot however long the leg is.
   */
  private static Path build(
      PathConstraints constraints,
      List<Translation2d> points,
      Rotation2d startHeading,
      Rotation2d endHeading,
      List<Turn> turns,
      Map<String, Double> eventsByRemainingMeters) {
    int segments = points.size() - 1;
    double[] lengths = new double[segments];
    double total = 0;
    for (int i = 0; i < segments; i++) {
      lengths[i] = points.get(i).getDistance(points.get(i + 1));
      total += lengths[i];
    }

    // Translation ordinals count the start waypoint as 0, then each later point in order. BLine
    // applies the limit for the point the robot is heading toward.
    var limits = new ArrayList<PathConstraints>();
    for (int i = 1; i + 1 < points.size(); i++) {
      var in = points.get(i).minus(points.get(i - 1));
      var out = points.get(i + 1).minus(points.get(i));
      double turnDegrees =
          Math.toDegrees(
              Math.abs(
                  Math.IEEEremainder(
                      Math.atan2(out.getY(), out.getX()) - Math.atan2(in.getY(), in.getX()),
                      2 * Math.PI)));
      if (turnDegrees > SHARP_CORNER_DEGREES) {
        limits.add(
            new PathConstraints()
                .setMaxVelocityMetersPerSec(CORNER_SPEED_METERS_PER_SEC)
                .inRange(i, i));
      }
    }
    limits.add(constraints);
    var combined = PathConstraints.combine(limits.toArray(PathConstraints[]::new));

    var elements = new ArrayList<PathElement>();
    elements.add(new Waypoint(points.get(0), startHeading));
    double traveled = 0;
    for (int i = 0; i < segments; i++) {
      for (var turn : turns) {
        if (turn.segment() == i) {
          elements.add(new RotationTarget(turn.heading(), turn.tRatio()));
        }
      }
      for (var event : eventsByRemainingMeters.entrySet()) {
        double at = Math.max(0, total - event.getValue()); // Distance from the start
        boolean lastSegment = i == segments - 1;
        if (at >= traveled && (at < traveled + lengths[i] || lastSegment)) {
          double tRatio = lengths[i] > 0 ? Math.clamp((at - traveled) / lengths[i], 0, 1) : 0;
          elements.add(new EventTrigger(tRatio, event.getKey()));
        }
      }
      traveled += lengths[i];
      var next = points.get(i + 1);
      elements.add(
          i == segments - 1 ? new Waypoint(next, endHeading) : new TranslationTarget(next));
    }
    return new Path(combined, elements.toArray(PathElement[]::new));
  }
}

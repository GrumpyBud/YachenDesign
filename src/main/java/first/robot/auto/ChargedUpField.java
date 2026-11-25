package first.robot.auto;

/**
 * 2023 Charged Up geometry on the blue side, in meters, plus this robot's clearances. Field values
 * are from Team 6328's 2023 FieldConstants (github.com/Mechanical-Advantage/RobotCode2023), which
 * match the WPILib 2023 tag layout. BLine mirrors everything for red.
 *
 * <p>Nodes are numbered 1 to 9 and staged pieces 1 to 4, both counting from the cable bump side (y
 * = 0) toward the loading zone side. Nodes 2, 5, and 8 are cube nodes. On red the numbers mean the
 * same physical positions, because the field is mirrored, not rotated.
 */
public final class ChargedUpField {
  private ChargedUpField() {}

  public static final int NODE_COUNT = 9;
  public static final int STAGED_PIECE_COUNT = 4;

  public static final double GRID_OUTER_X = 1.378;
  public static final double CHARGE_STATION_MIN_X = 2.919;
  public static final double CHARGE_STATION_MAX_X = 4.858;
  public static final double CHARGE_STATION_MIN_Y = 1.509;
  public static final double CHARGE_STATION_MAX_Y = 3.988;
  public static final double FIELD_WIDTH = 8.014;
  public static final double STAGED_PIECE_X = 7.068;

  private static final double FIRST_NODE_Y = 0.513;
  private static final double NODE_SEPARATION_Y = 0.559;
  private static final double FIRST_STAGED_PIECE_Y = 0.919;
  private static final double STAGED_PIECE_SEPARATION_Y = 1.219;

  // Robot geometry. Half-length with bumpers is an estimate from the CAD frame (28 in) plus
  // bumpers. Claw reach is the claw center ahead of the robot center at the CAD intake pose.
  public static final double HALF_LENGTH = 0.44;
  public static final double HALF_DIAGONAL = HALF_LENGTH * Math.sqrt(2);
  public static final double CLAW_REACH_AT_INTAKE = 0.586;

  /** Robot center x against the grid, with a few centimeters of clearance. */
  public static final double GRID_X = GRID_OUTER_X + HALF_LENGTH + 0.03;

  /**
   * Authored bumper clearance from the charge station in the lanes. BLine rounds corners inward by
   * up to its handoff radius (0.2 m), so this leaves room for that plus tracking error.
   */
  public static final double LANE_MARGIN = 0.3;

  /**
   * Lanes above and below the charge station. The robot never turns in a lane (the lower lane has
   * only about a third of a meter to the wall), so the bumpers' half-length is the clearance that
   * matters there. The lower lane crosses the cable bump.
   */
  public static final double UPPER_LANE_Y = CHARGE_STATION_MAX_Y + HALF_LENGTH + LANE_MARGIN;

  public static final double LOWER_LANE_Y = CHARGE_STATION_MIN_Y - HALF_LENGTH - LANE_MARGIN;

  /** West lane end: the robot, facing the grid, is {@link #LANE_MARGIN} clear of the station. */
  public static final double LANE_WEST_X = CHARGE_STATION_MIN_X - HALF_LENGTH - LANE_MARGIN;

  /** East lane end: past it, a turning robot's corners (half-diagonal) clear the station. */
  public static final double LANE_EAST_X = CHARGE_STATION_MAX_X + HALF_DIAGONAL + 0.05;

  /** Robot center x when the claw, at the intake position, is centered on a staged piece. */
  public static final double PIECE_CONTACT_X = STAGED_PIECE_X - CLAW_REACH_AT_INTAKE;

  /** Drive this far past contact so the rollers pull the piece in. */
  public static final double PICKUP_END_X = PIECE_CONTACT_X + 0.1;

  /**
   * Where the robot stops lined up behind a staged piece, then drives straight onto it. Far enough
   * east of the lane that the turn to face the field has a segment of its own.
   */
  public static final double APPROACH_X = PIECE_CONTACT_X - 0.6;

  /**
   * Where the robot stops behind a fallback piece. Closer than {@link #APPROACH_X}: the robot is
   * already facing the field, so it only needs room to lower the claw. With the claw stowed during
   * the sideways move, the claw stays clear of the pieces.
   */
  public static final double FALLBACK_APPROACH_X = PIECE_CONTACT_X - 0.35;

  public static double nodeY(int node) {
    checkNode(node);
    return FIRST_NODE_Y + (node - 1) * NODE_SEPARATION_Y;
  }

  public static boolean isCubeNode(int node) {
    checkNode(node);
    return node == 2 || node == 5 || node == 8;
  }

  public static double stagedPieceY(int piece) {
    checkPiece(piece);
    return FIRST_STAGED_PIECE_Y + (piece - 1) * STAGED_PIECE_SEPARATION_Y;
  }

  /** The lane that keeps a trip between two y positions shortest: above or below the station. */
  public static double laneBetween(double y1, double y2) {
    double stationCenterY = (CHARGE_STATION_MIN_Y + CHARGE_STATION_MAX_Y) / 2.0;
    return (y1 + y2) / 2.0 >= stationCenterY ? UPPER_LANE_Y : LOWER_LANE_Y;
  }

  public static void checkNode(int node) {
    if (node < 1 || node > NODE_COUNT) {
      throw new IllegalArgumentException("Node must be 1 to 9: " + node);
    }
  }

  public static void checkPiece(int piece) {
    if (piece < 1 || piece > STAGED_PIECE_COUNT) {
      throw new IllegalArgumentException("Staged piece must be 1 to 4: " + piece);
    }
  }
}

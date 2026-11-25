package first.robot.auto;

import static org.wpilib.units.Units.Seconds;

import first.robot.subsystems.Choreographer;
import first.robot.subsystems.Choreographer.Position;
import first.robot.subsystems.Choreographer.Source;
import first.robot.subsystems.claw.Claw;
import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Timer;

/**
 * Runs a {@link ScoringPlan}: score the preload, pick up a staged piece (trying fallbacks if the
 * claw does not detect it), and score it.
 *
 * <p>Tip safety: the mechanisms are fully retracted (stowed and confirmed) before every drive, and
 * only extend near the end of a leg as it slows to a stop (see {@link CyclePaths}). The one
 * exception is driving onto a piece, which happens slowly with the claw at the low intake position.
 */
final class ScoringAuto {
  private ScoringAuto() {}

  private static final double POSITION_TIMEOUT_SECONDS = 5;
  private static final double EJECT_SECONDS = 0.3;
  private static final double PICKUP_CONFIRM_SECONDS = 0.25;

  /**
   * Latest routine time to start another pickup attempt. An attempt (retract, move, drive onto the
   * piece) takes about 3.5 s in simulation, so it ends inside the 15 s routine.
   */
  private static final double LATEST_PICKUP_START_SECONDS = 9.5;

  /** The routine's limit in Autos, and the margin to keep under it. */
  private static final double ROUTINE_SECONDS = 15.0;

  private static final double ROUTINE_MARGIN_SECONDS = 0.5;

  // Return time model, measured in simulation: about 1.3 s of fixed time (finishing the raise,
  // ejecting) plus the return paths' length at about 2 m/s average. 5.3 m measured 4.0 s.
  private static final double RETURN_FIXED_SECONDS = 1.3;
  private static final double RETURN_AVERAGE_METERS_PER_SEC = 2.0;

  static void run(AutoContext auto, Choreographer robot, ScoringPlan plan) {
    double start = Timer.getTimestamp();
    Logger.recordOutput("Auto/Plan", plan.toString());

    // Set odometry to the start before scoring, so the preload is scored at the real pose.
    if (!auto.startAt(CyclePaths.gridToPiece(plan.startNode(), plan.piece()))) return;
    if (!score(auto, robot, "preload")) return;

    Integer heldFrom = null; // The staged piece now in the claw
    Integer lastTried = null; // Where the robot is: behind this piece, or at the grid if null
    for (int piece : plan.piecesToTry()) {
      if (lastTried != null && Timer.getTimestamp() - start > LATEST_PICKUP_START_SECONDS) {
        Logger.recordOutput("Auto/PickupStatus", "No time left for fallback piece " + piece);
        break;
      }
      Logger.recordOutput("Auto/PickupStatus", "Trying piece " + piece);
      if (!retract(auto, robot)) return;
      var toPiece =
          lastTried == null
              ? CyclePaths.gridToPiece(plan.startNode(), piece)
              : CyclePaths.pieceToPiece(lastTried, piece);
      // The path's event lowers the intake and starts the rollers just before it stops.
      if (!auto.follow(toPiece, 5.0)) return;
      if (!reach(auto, robot, Position.INTAKE)) return;
      var onto =
          lastTried == null ? CyclePaths.ontoPiece(piece) : CyclePaths.ontoFallbackPiece(piece);
      if (!auto.follow(onto, 2.5)) return;
      lastTried = piece;
      // Missing a piece is not a fault yet: the next fallback may still work.
      boolean detected =
          !auto.coroutine()
              .waitUntil(robot::hasGamePiece, Seconds.of(PICKUP_CONFIRM_SECONDS))
              .timedOut();
      robot.requestClaw(Source.AUTO, Claw.Request.STOP);
      if (detected) {
        heldFrom = piece;
        break;
      }
      Logger.recordOutput("Auto/PickupStatus", "Missed piece " + piece);
    }
    if (heldFrom == null) {
      retract(auto, robot);
      auto.fail("No game piece detected after trying pieces " + plan.piecesToTry());
      return;
    }
    Logger.recordOutput("Auto/PickupStatus", "Holding piece " + heldFrom);

    if (!retract(auto, robot)) return;
    System.out.printf("PHASE return-start %.2f%n", Timer.getTimestamp() - start);
    var toLane = CyclePaths.pieceToLane(heldFrom, plan.scoreNode());
    var toNode = CyclePaths.laneToNode(heldFrom, plan.scoreNode());
    double returnSeconds =
        RETURN_FIXED_SECONDS
            + (CyclePaths.length(toLane) + CyclePaths.length(toNode))
                / RETURN_AVERAGE_METERS_PER_SEC;
    double elapsed = Timer.getTimestamp() - start;
    Logger.recordOutput("Auto/ReturnEstimateSeconds", returnSeconds);
    if (elapsed + returnSeconds > ROUTINE_SECONDS - ROUTINE_MARGIN_SECONDS) {
      // Not a fault: the piece is in the claw for teleop, and the robot is retracted.
      Logger.recordOutput("Auto/Result", "Holding piece " + heldFrom + "; no time left to score");
      return;
    }
    if (!auto.follow(toLane, 5.0)) return;
    // The path's event raises the elevator just before it stops at the node.
    if (!auto.follow(toNode, 4.0)) return;
    if (!score(auto, robot, "piece " + heldFrom)) return;

    // Leave it retracting; nothing moves after this, and the position stays requested.
    robot.requestPosition(Source.AUTO, Position.STOW);
    Logger.recordOutput("Auto/Result", "Scored the preload and piece " + heldFrom);
    System.out.printf("PHASE done %.2f%n", Timer.getTimestamp() - start);
  }

  /** Raise to score if not already there, wait for the joints, eject, then retract fully. */
  private static boolean score(AutoContext auto, Choreographer robot, String what) {
    robot.requestPosition(Source.AUTO, Position.SCORE);
    if (!reach(auto, robot, Position.SCORE)) return false;
    robot.requestClaw(Source.AUTO, Claw.Request.EJECT);
    auto.waitSeconds(EJECT_SECONDS);
    robot.requestClaw(Source.AUTO, Claw.Request.STOP);
    Logger.recordOutput("Auto/LastScored", what);
    return true;
  }

  /** Stow and wait until the joints are there. Every drive starts from here. */
  private static boolean retract(AutoContext auto, Choreographer robot) {
    robot.requestClaw(Source.AUTO, Claw.Request.STOP);
    robot.requestPosition(Source.AUTO, Position.STOW);
    return reach(auto, robot, Position.STOW);
  }

  /** Wait until the elevator and arm reach an already requested position. */
  private static boolean reach(AutoContext auto, Choreographer robot, Position position) {
    if (robot.getPosition() != position) {
      return auto.fail("Expected " + position + " but " + robot.getPosition() + " is requested");
    }
    return auto.waitUntil(position + " position", robot::atPosition, POSITION_TIMEOUT_SECONDS);
  }
}

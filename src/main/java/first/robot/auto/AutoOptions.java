package first.robot.auto;

import java.util.ArrayList;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;

/**
 * Dashboard choosers for the "2023: Custom" routine. Choose them before enabling autonomous. The
 * choices are logged, so a replay uses the same plan. Nodes and pieces count from the cable bump
 * side, and mean the same positions on either alliance.
 */
final class AutoOptions {
  private static final int NONE = 0;

  private final LoggedNetworkChooser<Integer> startNode = nodeChooser("Auto Start Node", 8);
  private final LoggedNetworkChooser<Integer> piece = pieceChooser("Auto Pickup", 4, false);
  private final LoggedNetworkChooser<Integer> scoreNode = nodeChooser("Auto Score Node", 7);
  private final LoggedNetworkChooser<Integer> fallback1 =
      pieceChooser("Auto Fallback 1", NONE, true);
  private final LoggedNetworkChooser<Integer> fallback2 =
      pieceChooser("Auto Fallback 2", NONE, true);

  /** The plan from the current choices. */
  ScoringPlan plan() {
    var fallbacks = new ArrayList<Integer>();
    for (var chooser : new LoggedNetworkChooser<?>[] {fallback1, fallback2}) {
      Object choice = chooser.get();
      if (choice instanceof Integer value && value != NONE) {
        fallbacks.add(value);
      }
    }
    return new ScoringPlan(
        orDefault(startNode.get(), 8),
        orDefault(piece.get(), 4),
        orDefault(scoreNode.get(), 7),
        fallbacks);
  }

  private static int orDefault(Integer value, int fallback) {
    return value == null ? fallback : value;
  }

  private static LoggedNetworkChooser<Integer> nodeChooser(String key, int defaultNode) {
    var chooser = new LoggedNetworkChooser<Integer>(key);
    for (int node = 1; node <= ChargedUpField.NODE_COUNT; node++) {
      String label = nodeLabel(node);
      if (node == defaultNode) {
        chooser.addDefault(label, node);
      } else {
        chooser.add(label, node);
      }
    }
    return chooser;
  }

  private static LoggedNetworkChooser<Integer> pieceChooser(
      String key, int defaultPiece, boolean allowNone) {
    var chooser = new LoggedNetworkChooser<Integer>(key);
    if (allowNone) {
      if (defaultPiece == NONE) {
        chooser.addDefault("None", NONE);
      } else {
        chooser.add("None", NONE);
      }
    }
    for (int piece = 1; piece <= ChargedUpField.STAGED_PIECE_COUNT; piece++) {
      String label = pieceLabel(piece);
      if (piece == defaultPiece) {
        chooser.addDefault(label, piece);
      } else {
        chooser.add(label, piece);
      }
    }
    return chooser;
  }

  static String nodeLabel(int node) {
    String side =
        node == 1 ? ", bump side" : node == ChargedUpField.NODE_COUNT ? ", loading zone side" : "";
    return "Node " + node + " (" + (ChargedUpField.isCubeNode(node) ? "cube" : "cone") + side + ")";
  }

  static String pieceLabel(int piece) {
    String side =
        piece == 1
            ? " (bump side)"
            : piece == ChargedUpField.STAGED_PIECE_COUNT ? " (loading zone side)" : "";
    return "Piece " + piece + side;
  }
}

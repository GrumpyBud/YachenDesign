package first.robot.auto;

import java.util.ArrayList;
import java.util.List;

/**
 * A preload-plus-one-pickup autonomous, described by field locations instead of hand-drawn paths.
 *
 * @param startNode node 1 to 9 to start against and score the preload on
 * @param piece staged piece 1 to 4 to pick up first
 * @param scoreNode node 1 to 9 to score the picked-up piece on
 * @param fallbacks staged pieces to try, in order, if the pickup misses; may be empty
 */
public record ScoringPlan(int startNode, int piece, int scoreNode, List<Integer> fallbacks) {
  public ScoringPlan {
    ChargedUpField.checkNode(startNode);
    ChargedUpField.checkPiece(piece);
    ChargedUpField.checkNode(scoreNode);
    // Drop repeats and the primary piece, keeping the order the operator chose.
    var cleaned = new ArrayList<Integer>();
    for (int fallback : fallbacks) {
      ChargedUpField.checkPiece(fallback);
      if (fallback != piece && !cleaned.contains(fallback)) {
        cleaned.add(fallback);
      }
    }
    fallbacks = List.copyOf(cleaned);
  }

  /** The primary piece, then each fallback. */
  public List<Integer> piecesToTry() {
    var pieces = new ArrayList<Integer>();
    pieces.add(piece);
    pieces.addAll(fallbacks);
    return pieces;
  }

  @Override
  public String toString() {
    return "start node "
        + startNode
        + ", piece "
        + piece
        + (fallbacks.isEmpty() ? "" : " (fallbacks " + fallbacks + ")")
        + ", score node "
        + scoreNode;
  }
}

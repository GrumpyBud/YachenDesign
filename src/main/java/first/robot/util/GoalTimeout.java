package first.robot.util;

import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Flags a closed-loop mechanism that has been driving toward the same goal for too long without
 * reaching it. The timer restarts when the goal changes or the mechanism resumes after neutral. It
 * only reports; the mechanism keeps holding its goal.
 */
public final class GoalTimeout {
  private final Alert alert;
  private double startedAt = Double.NaN;
  private double lastGoal = Double.NaN;
  private boolean timedOut = false;

  public GoalTimeout(String alertGroup, String text) {
    alert = new Alert(alertGroup, text, Level.MEDIUM);
  }

  /** Update once per loop and return whether the current goal has timed out. */
  public boolean update(boolean running, double goal, boolean atGoal, double timeoutSeconds) {
    double now = Timer.getTimestamp();
    if (!running) {
      startedAt = Double.NaN;
    } else if (Double.isNaN(startedAt) || goal != lastGoal) {
      startedAt = now;
    }
    lastGoal = goal;
    timedOut = running && !atGoal && now - startedAt > timeoutSeconds;
    alert.set(timedOut);
    return timedOut;
  }

  /** Seconds spent on the current goal, or 0 while not running. */
  public double elapsedSeconds() {
    return Double.isNaN(startedAt) ? 0.0 : Timer.getTimestamp() - startedAt;
  }
}

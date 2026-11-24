package first.robot.auto;

import static org.wpilib.units.Units.Seconds;

import bline.commands.FollowPath;
import bline.path.Path;
import java.io.File;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;

/**
 * The tools a routine body uses while it runs. Each step returns {@code false} after recording a
 * fault, so routines read as:
 *
 * <pre>{@code
 * if (!auto.follow(AutoPaths.toFirstPiece())) return;
 * if (!auto.await(intake.collect())) return;
 * auto.follow(AutoPaths.backToScore());
 * }</pre>
 *
 * <p>The first path a routine follows resets odometry to its authored start (flipped for red).
 * Later paths start from the measured pose, so their first element should be where the previous
 * path ended.
 */
public final class AutoContext {
  private final Coroutine co;
  private final FollowPath.Builder follower;
  private final File autosDirectory;
  private final BooleanSupplier pathFinished;
  private final Runnable clearPathFinished;
  private int segment;
  private String failure;

  AutoContext(
      Coroutine co,
      FollowPath.Builder follower,
      File autosDirectory,
      BooleanSupplier pathFinished,
      Runnable clearPathFinished) {
    this.co = co;
    this.follower = follower;
    this.autosDirectory = autosDirectory;
    this.pathFinished = pathFinished;
    this.clearPathFinished = clearPathFinished;
  }

  /**
   * Follows a path until BLine reports the endpoint within tolerance.
   *
   * @return false, with a fault recorded, if the path is invalid or the robot did not arrive
   */
  public boolean follow(Path path) {
    return follow(path, "segment " + (segment + 1));
  }

  /**
   * Follows a path with a per-segment timeout, so one stuck segment cannot use the whole routine.
   *
   * @return false, with a fault recorded, if the path is invalid, timed out, or did not arrive
   */
  public boolean follow(Path path, double timeoutSeconds) {
    return follow(path, "segment " + (segment + 1), timeoutSeconds);
  }

  /**
   * Loads a path drawn in BLine Web from {@code deploy/autos/paths/<name>.json} and follows it.
   *
   * @param name file name without {@code .json}
   */
  public boolean followDrawn(String name) {
    Path path;
    try {
      path = new Path(autosDirectory, name);
    } catch (RuntimeException error) {
      return fail("Could not load drawn path " + name + ": " + error.getMessage());
    }
    return follow(path, name);
  }

  private boolean follow(Path path, String label) {
    return follow(path, label, Double.POSITIVE_INFINITY);
  }

  private boolean follow(Path path, String label, double timeoutSeconds) {
    if (failure != null) {
      return false;
    }
    segment++;
    if (!path.isValid()) {
      return fail("Invalid path: " + label);
    }
    boolean first = segment == 1;
    if (first && path.getAuthoredStartPose().isEmpty()) {
      return fail("The first path must start with a waypoint: " + label);
    }

    Logger.recordOutput("Auto/Segment", label);
    FollowPath built = follower.build(path);
    FollowPath command = first ? built.withPoseReset() : built;
    Command segmentCommand =
        Double.isFinite(timeoutSeconds) ? command.withTimeout(Seconds.of(timeoutSeconds)) : command;

    clearPathFinished.run();
    if (!co.await(segmentCommand).successful()) {
      return fail("Could not start " + label);
    }
    if (!pathFinished.getAsBoolean()) {
      return fail("Path did not reach its endpoint: " + label);
    }
    return true;
  }

  /** Pauses the routine. The drivetrain receives no path output while waiting. */
  public void waitSeconds(double seconds) {
    if (failure == null) {
      co.wait(Seconds.of(seconds));
    }
  }

  /**
   * Runs a mechanism command to completion. This only proves the command ended. Have the command
   * itself report or check whether the mechanism succeeded, and call {@link #fail} if it did not.
   *
   * @return false if the command could not be scheduled
   */
  public boolean await(Command command) {
    if (failure != null) {
      return false;
    }
    if (!co.await(command).successful()) {
      return fail("Could not start " + command.name());
    }
    return true;
  }

  /**
   * Waits until a condition is true, such as a game piece sensor, or until the timeout.
   *
   * @return false, with a fault recorded, if the timeout expired first
   */
  public boolean waitUntil(String description, BooleanSupplier condition, double timeoutSeconds) {
    if (failure != null) {
      return false;
    }
    if (co.waitUntil(condition, Seconds.of(timeoutSeconds)).timedOut()) {
      return fail("Timed out waiting for " + description);
    }
    return true;
  }

  /** Records a fault and stops further steps. Always returns false. */
  public boolean fail(String message) {
    if (failure == null) {
      failure = message;
    }
    return false;
  }

  /**
   * The underlying coroutine, for Commands v3 features such as {@code fork} or {@code awaitAll}.
   */
  public Coroutine coroutine() {
    return co;
  }

  String failure() {
    return failure;
  }
}

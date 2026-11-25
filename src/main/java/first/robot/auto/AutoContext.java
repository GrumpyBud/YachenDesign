package first.robot.auto;

import static org.wpilib.units.Units.Seconds;

import bline.commands.FollowPath;
import bline.field.FlippingUtil;
import bline.path.Path;
import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.math.geometry.Pose2d;

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
 * <p>The first path a routine follows resets odometry to its authored start (flipped for red),
 * unless the routine already called {@link #startAt}. Later paths start from the measured pose, so
 * their first element should be where the previous path ended.
 */
public final class AutoContext {
  private final Coroutine co;
  private final FollowPath.Builder follower;
  private final File autosDirectory;
  private final BooleanSupplier pathFinished;
  private final Runnable clearPathFinished;
  private final Consumer<Pose2d> resetPose;
  private final BooleanSupplier redAlliance;
  private boolean poseReset;
  private int segment;
  private String failure;

  AutoContext(
      Coroutine co,
      FollowPath.Builder follower,
      File autosDirectory,
      BooleanSupplier pathFinished,
      Runnable clearPathFinished,
      Consumer<Pose2d> resetPose,
      BooleanSupplier redAlliance) {
    this.co = co;
    this.resetPose = resetPose;
    this.redAlliance = redAlliance;
    this.follower = follower;
    this.autosDirectory = autosDirectory;
    this.pathFinished = pathFinished;
    this.clearPathFinished = clearPathFinished;
  }

  /**
   * Resets odometry to a path's authored start (flipped for red) without driving. Use it when the
   * routine acts before its first path, such as scoring a preload, so that work happens at the real
   * starting pose. The next {@link #follow} then starts from the measured pose.
   *
   * @return false, with a fault recorded, if the path has no authored start
   */
  public boolean startAt(Path path) {
    if (failure != null) {
      return false;
    }
    var start = path.getAuthoredStartPose();
    if (start.isEmpty()) {
      return fail("startAt needs a path that begins with a waypoint");
    }
    var pose = redAlliance.getAsBoolean() ? FlippingUtil.flipFieldPose(start.get()) : start.get();
    resetPose.accept(pose);
    poseReset = true;
    Logger.recordOutput("Auto/StartPose", pose);
    return true;
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
    boolean first = !poseReset;
    if (first && path.getAuthoredStartPose().isEmpty()) {
      return fail("The first path must start with a waypoint: " + label);
    }
    poseReset = true;

    Logger.recordOutput("Auto/Segment", label);
    // BLine's alliance flip always rotates the path about the field center. On a mirrored field
    // (2023), red is a reflection across the center line, which is that rotation plus BLine's
    // width mirror: (x, y, theta) -> (length - x, y, pi - theta).
    FollowPath built = follower.build(path).withShouldMirror(this::mirrorForRed);
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

  private boolean mirrorForRed() {
    return redAlliance.getAsBoolean()
        && FlippingUtil.symmetryType == FlippingUtil.FieldSymmetry.kMirrored;
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

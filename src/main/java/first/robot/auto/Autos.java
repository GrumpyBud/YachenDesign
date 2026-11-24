package first.robot.auto;

import java.util.List;

/**
 * Autonomous routines defined in Java. This is the file to edit when adding an auto.
 *
 * <p>Paths drawn in BLine Web need no code: export them into {@code deploy/autos/paths/} and they
 * appear in the chooser as "Drawn: name". A Java routine can also use a drawn path with {@link
 * AutoContext#followDrawn}.
 */
public final class Autos {
  private Autos() {}

  /** Routines in chooser order. "Do Nothing" is added first automatically and is the default. */
  static List<AutoRoutine> routines() {
    return List.of(
        AutoRoutine.singlePath("Drive 1 Meter", 5, AutoPaths::drive1Meter),
        AutoRoutine.singlePath("Strafe 1 Meter (Calibration)", 5, AutoPaths::strafe1Meter),
        AutoRoutine.singlePath("Turn 90 Degrees (Calibration)", 5, AutoPaths::turn90Degrees),
        new AutoRoutine(
            "Example: Out and Back",
            12,
            auto -> {
              if (!auto.follow(AutoPaths.drive1Meter())) return;
              auto.waitSeconds(0.5);
              auto.follow(AutoPaths.driveBack1Meter());
            }));
  }

  /**
   * Connects event names used by {@code EventTrigger}s, in Java or drawn paths, to robot actions.
   * BLine only logs a warning for an unregistered name, so register every name a path uses.
   *
   * <p>A {@code Runnable} must return immediately. A {@code Command} is scheduled on its own and
   * keeps running after the path ends, so give it its own end condition. Example:
   *
   * <pre>{@code
   * FollowPath.registerEventTrigger("deployIntake", intake::deploy);
   * FollowPath.registerEventTrigger("spinUp", shooter.spinUpCommand());
   * }</pre>
   */
  static void registerEvents() {
    // No mechanisms yet. Pass them in from RobotContainer and register their events here.
  }
}

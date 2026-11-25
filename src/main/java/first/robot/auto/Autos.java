package first.robot.auto;

import bline.commands.FollowPath;
import first.robot.subsystems.Choreographer;
import first.robot.subsystems.Choreographer.Position;
import first.robot.subsystems.Choreographer.Source;
import first.robot.subsystems.claw.Claw;
import java.util.List;

/**
 * Autonomous routines defined in Java. This is the file to edit when adding an auto.
 *
 * <p>Scoring autos are {@link ScoringPlan}s: pick the start node, staged piece, scoring node, and
 * fallback pieces, and {@link CyclePaths} builds the paths. "2023: Custom" takes its plan from
 * dashboard choosers. Paths drawn in BLine Web need no code: export them into {@code
 * deploy/autos/paths/} and they appear in the chooser as "Drawn: name".
 *
 * <p>Routines move the mechanisms through the {@link Choreographer} as {@link Source#AUTO}, which
 * only works during autonomous; the claw stops when the routine or autonomous ends.
 */
public final class Autos {
  private Autos() {}

  /** Event names for {@code EventTrigger}s in Java or drawn paths. */
  public static final String STOW = "stow";

  public static final String INTAKE = "intake";
  public static final String SCORE = "score";

  /**
   * Routines in chooser order. "Do Nothing" is added first automatically and is the default.
   *
   * @param robot the mechanism coordinator; null only when the build validates paths, which never
   *     runs a routine body
   * @param options dashboard plan choosers; null under the same condition
   */
  static List<AutoRoutine> routines(Choreographer robot, AutoOptions options) {
    return List.of(
        new AutoRoutine(
            "2023: Custom (dashboard)", 15, auto -> ScoringAuto.run(auto, robot, options.plan())),
        scoring(robot, "2023: Node 8, Piece 4, Node 7", new ScoringPlan(8, 4, 7, List.of())),
        scoring(
            robot,
            "2023: Node 8, Piece 4 (fallback 3), Node 7",
            new ScoringPlan(8, 4, 7, List.of(3))),
        scoring(
            robot,
            "2023: Node 2, Piece 1 (fallback 2), Node 3",
            new ScoringPlan(2, 1, 3, List.of(2))),
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

  /** A scoring routine with a fixed plan. */
  static AutoRoutine scoring(Choreographer robot, String name, ScoringPlan plan) {
    return new AutoRoutine(name, 15, auto -> ScoringAuto.run(auto, robot, plan));
  }

  /**
   * Connects event names used by {@code EventTrigger}s, in Java or drawn paths, to robot actions.
   * BLine only logs a warning for an unregistered name, so register every name a path uses.
   *
   * <p>A {@code Runnable} must return immediately. A {@code Command} is scheduled on its own and
   * keeps running after the path ends, so give it its own end condition.
   */
  static void registerEvents(Choreographer robot) {
    FollowPath.registerEventTrigger(
        STOW,
        () -> {
          robot.requestPosition(Source.AUTO, Position.STOW);
          robot.requestClaw(Source.AUTO, Claw.Request.STOP);
        });
    FollowPath.registerEventTrigger(
        INTAKE,
        () -> {
          robot.requestPosition(Source.AUTO, Position.INTAKE);
          robot.requestClaw(Source.AUTO, Claw.Request.INTAKE);
        });
    FollowPath.registerEventTrigger(
        SCORE, () -> robot.requestPosition(Source.AUTO, Position.SCORE));
  }
}

package first.robot.auto;

import bline.path.Path;
import java.util.function.Supplier;

/**
 * One entry in the dashboard's Auto Routine chooser.
 *
 * <p>The body runs as a Commands v3 coroutine. Use the {@link AutoContext} it receives to follow
 * paths, wait, and run mechanism commands in order. Stop early with {@code return} when a step
 * reports failure, so a missed step never turns into a silent success.
 *
 * @param name chooser label; must be unique
 * @param timeoutSeconds whole-routine limit; the routine is cancelled and reported when exceeded
 * @param body the steps to run
 */
public record AutoRoutine(String name, double timeoutSeconds, Body body) {
  /** The steps of a routine. */
  @FunctionalInterface
  public interface Body {
    void run(AutoContext auto);
  }

  public AutoRoutine {
    if (name == null || name.isBlank()) {
      throw new IllegalArgumentException("Routine name must not be blank");
    }
    if (!(timeoutSeconds > 0)) {
      throw new IllegalArgumentException(name + ": timeout must be positive");
    }
  }

  /** A routine that follows one Java-defined path, resetting to its authored start. */
  public static AutoRoutine singlePath(String name, double timeoutSeconds, Supplier<Path> path) {
    return new AutoRoutine(name, timeoutSeconds, auto -> auto.follow(path.get()));
  }
}

package first.robot.sim;

import first.robot.subsystems.arm.ArmConstants;
import first.robot.subsystems.elevator.ElevatorConstants;
import first.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.system.Timer;

/**
 * Simulation-only 2023 game pieces: the eight staged pieces and the robot's preload. The claw picks
 * up a staged piece when it is low, close to the piece, and intaking, and releases the held piece
 * after ejecting briefly. Used only by {@code ClawIOSim} to load the rollers, so the real
 * current-based detection can be exercised. This is a geometric approximation, not physics.
 */
public final class SimGamePieces {
  // Staged pieces, 2023 (Team 6328 FieldConstants): 47.36 in from the center line, first at
  // 36.19 in from the field edge, 48 in apart.
  private static final double STAGED_X_FROM_CENTER = 1.203;
  private static final double STAGED_FIRST_Y = 0.919;
  private static final double STAGED_SEPARATION_Y = 1.219;
  private static final double PIECE_HEIGHT = 0.12; // Center height of a resting piece

  private static final double PICKUP_RADIUS = 0.15; // Claw center to piece, horizontally
  private static final double PICKUP_MAX_CLAW_HEIGHT = 0.35;
  private static final double RELEASE_SECONDS = 0.15;

  private static final Translation3d ELEVATOR_AXIS =
      new Translation3d(
          Math.cos(ElevatorConstants.TILT_RAD), 0.0, Math.sin(ElevatorConstants.TILT_RAD));

  private final Supplier<Pose2d> robotPose;
  private final DoubleSupplier elevatorMeters;
  private final DoubleSupplier armRad;
  private final List<Translation2d> staged = new ArrayList<>();
  private boolean held;
  private double ejectingSince = Double.NaN;
  private int released;

  /**
   * @param robotPose the simulated ground-truth pose, not the estimated one
   */
  public SimGamePieces(
      Supplier<Pose2d> robotPose, DoubleSupplier elevatorMeters, DoubleSupplier armRad) {
    this.robotPose = robotPose;
    this.elevatorMeters = elevatorMeters;
    this.armRad = armRad;
    reset();
  }

  /** Restores all eight staged pieces and a preload in the claw. */
  public void reset() {
    staged.clear();
    double fieldLength = VisionConstants.aprilTagLayout.getFieldLength();
    double center = fieldLength / 2.0;
    for (int i = 0; i < 4; i++) {
      double y = STAGED_FIRST_Y + i * STAGED_SEPARATION_Y;
      staged.add(new Translation2d(center - STAGED_X_FROM_CENTER, y)); // Blue side
      staged.add(new Translation2d(center + STAGED_X_FROM_CENTER, y)); // Red side
    }
    held = true;
    ejectingSince = Double.NaN;
    released = 0;
  }

  /** Removes the staged piece nearest a field position, for testing a missed pickup. */
  public void removeStagedNear(double x, double y) {
    var target = new Translation2d(x, y);
    staged.stream()
        .min((p, q) -> Double.compare(p.getDistance(target), q.getDistance(target)))
        .ifPresent(staged::remove);
  }

  /** Removes the staged pieces, for testing a missed pickup. */
  public void clearStaged() {
    staged.clear();
  }

  /** Pieces released from the claw since the last reset. */
  public int releasedCount() {
    return released;
  }

  /**
   * Advances the model one loop.
   *
   * @param rollerVolts the voltage applied to the claw rollers; positive intakes
   * @return whether a piece is in the claw
   */
  public boolean update(double rollerVolts) {
    var claw = clawPosition();
    if (!held && rollerVolts > 1.0 && claw.getZ() < PICKUP_MAX_CLAW_HEIGHT) {
      var clawXY = claw.toTranslation2d();
      for (int i = 0; i < staged.size(); i++) {
        if (staged.get(i).getDistance(clawXY) < PICKUP_RADIUS) {
          staged.remove(i);
          held = true;
          break;
        }
      }
    }
    if (held && rollerVolts < -1.0) {
      double now = Timer.getTimestamp();
      if (Double.isNaN(ejectingSince)) {
        ejectingSince = now;
      } else if (now - ejectingSince >= RELEASE_SECONDS) {
        held = false;
        released++;
        ejectingSince = Double.NaN;
      }
    } else {
      ejectingSince = Double.NaN;
    }

    Logger.recordOutput(
        "SimField/StagedPieces",
        staged.stream()
            .map(t -> new Pose3d(t.getX(), t.getY(), PIECE_HEIGHT, new Rotation3d()))
            .toArray(Pose3d[]::new));
    Logger.recordOutput(
        "SimField/HeldPiece",
        held ? new Pose3d[] {new Pose3d(claw, new Rotation3d())} : new Pose3d[0]);
    Logger.recordOutput("SimField/ReleasedPieces", released);
    return held;
  }

  /** Claw center in field coordinates, from the robot pose and measured joint positions. */
  private Translation3d clawPosition() {
    double arm = armRad.getAsDouble();
    var robotRelative =
        ArmConstants.PIVOT_AT_ZERO_TRAVEL
            .plus(ELEVATOR_AXIS.times(elevatorMeters.getAsDouble()))
            .plus(
                new Translation3d(
                    ArmConstants.PIVOT_TO_CLAW_METERS * Math.cos(arm),
                    0.0,
                    ArmConstants.PIVOT_TO_CLAW_METERS * Math.sin(arm)));
    var pose = robotPose.get();
    var xy =
        new Translation2d(robotRelative.getX(), robotRelative.getY())
            .rotateBy(pose.getRotation())
            .plus(pose.getTranslation());
    return new Translation3d(xy.getX(), xy.getY(), robotRelative.getZ());
  }
}

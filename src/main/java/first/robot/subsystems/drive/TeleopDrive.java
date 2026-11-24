package first.robot.subsystems.drive;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;

/** Converts driver stick values into field-relative drive velocities. */
public final class TeleopDrive {
  private static final double DEADBAND = 0.10;
  private final Drive drive;
  private Rotation2d driverHeadingOffset = Rotation2d.ZERO;

  public TeleopDrive(Drive drive) {
    this.drive = drive;
  }

  /** Make the current heading the driver's forward direction. The field pose is unchanged. */
  public void setDriverForward() {
    driverHeadingOffset = drive.getRotation();
  }

  /** Return to the field's forward direction. */
  public void resetDriverForward() {
    driverHeadingOffset = Rotation2d.ZERO;
  }

  /** Drive from stick values in [-1, 1]: forward, left, and counterclockwise rotation. */
  public void run(double forward, double left, double rotation) {
    var fieldRelative =
        new ChassisVelocities(
            shape(forward) * drive.getMaxLinearSpeedMetersPerSec(),
            shape(left) * drive.getMaxLinearSpeedMetersPerSec(),
            shape(rotation) * drive.getMaxAngularSpeedRadPerSec());
    drive.runVelocity(
        fieldRelative.toRobotRelative(drive.getRotation().minus(driverHeadingOffset)));
  }

  private static double shape(double value) {
    double scaled = MathUtil.applyDeadband(value, DEADBAND);
    return Math.copySign(scaled * scaled, scaled);
  }
}

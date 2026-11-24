// Copyright (c) 2021-2026 Littleton Robotics
// SPDX-License-Identifier: BSD-3-Clause

package first.robot.subsystems.vision;

import java.util.List;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

public final class VisionConstants {
  private VisionConstants() {}

  // 2023 Charged Up field. Set the actual field before enabling real cameras.
  public static final Field aprilTagLayout = Fields.FRC_2023_CHARGED_UP.loadField();

  public record PhotonCameraConfig(String name, Transform3d robotToCamera) {}

  // Add one entry per Luma P1 camera with its PhotonVision nickname and measured mount.
  public static final List<PhotonCameraConfig> REAL_CAMERAS = List.of();
  public static final PhotonCameraConfig SIM_CAMERA =
      new PhotonCameraConfig(
          "sim-front", new Transform3d(0.2, 0, 0.5, new Rotation3d(0, -0.15, 0)));

  static final double maxAmbiguity = 0.3;
  static final double maxZError = 0.75;
  static final double linearStdDevBaseline = 0.02; // Meters at one meter and one tag.
  static final double angularStdDevBaseline = 0.06; // Radians at one meter and one tag.
  static final double[] cameraStdDevFactors = {1.0};
  static final double linearStdDevMegatag2Factor = 0.5;
  static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}

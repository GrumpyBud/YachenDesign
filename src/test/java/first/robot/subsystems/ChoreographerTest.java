package first.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.subsystems.Choreographer.Position;
import first.robot.subsystems.arm.ArmIOInputsAutoLogged;
import first.robot.subsystems.arm.ArmIOSim;
import first.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
import first.robot.subsystems.elevator.ElevatorIOSim;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

class ChoreographerTest {
  @BeforeAll
  static void initializeHal() {
    // The physics sims read the simulated battery voltage through the HAL.
    assertTrue(HAL.initialize());
  }

  /**
   * Component motions measured between the CAD exports, converted to the robot frame (x = -CAD y, z
   * = CAD z). Each entry is {x, z, pitch degrees} for stage, carriage, and arm.
   */
  private static final double[][] SCORE_CAD = {
    {0.536, 0.448, 0.0}, {1.0027, 0.8398, 0.0}, {0.878, 0.824, 25.52}
  };

  private static final double[][] INTAKE_CAD = {
    {0.0, 0.0, 0.0}, {0.172, 0.144, 0.0}, {-0.112, 0.197, 61.14}
  };

  private static final double[][] STOW_CAD = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  /**
   * AdvantageScope draws each component model at (logged pose) then (zeroed transform from
   * config.json). Starting from the models' stowed CAD placement, that must reproduce the motion
   * measured between the CAD exports.
   */
  @Test
  void renderedComponentsMatchCadExports() throws IOException {
    var zeroed = zeroedTransforms();
    assertEquals(3, zeroed.size());
    check(Position.STOW, STOW_CAD, zeroed);
    check(Position.SCORE, SCORE_CAD, zeroed);
    check(Position.INTAKE, INTAKE_CAD, zeroed);
  }

  @Test
  void simulatedJointsReachEveryPosition() {
    var elevator = new ElevatorIOSim();
    var arm = new ArmIOSim();
    elevator.configure(40.0, 1.0, 2.5, 8.0);
    arm.configure(30.0, 1.0, 6.0, 20.0);
    var elevatorInputs = new ElevatorIOInputsAutoLogged();
    var armInputs = new ArmIOInputsAutoLogged();
    for (var position : new Position[] {Position.SCORE, Position.INTAKE, Position.STOW}) {
      double armGoal = Math.toRadians(position.defaultArmDegrees);
      for (int loop = 0; loop < 150; loop++) { // 3 s
        elevator.runPosition(position.defaultElevatorMeters);
        arm.runPosition(armGoal);
        elevator.updateInputs(elevatorInputs);
        arm.updateInputs(armInputs);
      }
      assertEquals(
          position.defaultElevatorMeters, elevatorInputs.positionMeters, 0.02, position + "");
      assertEquals(armGoal, armInputs.positionRad, Math.toRadians(2.0), position + "");
      assertTrue(Math.abs(elevatorInputs.appliedVolts[0]) < 12.0);
    }
  }

  private static Pose3d[] poses(Position position) {
    return Choreographer.componentPoses(
        position.defaultElevatorMeters, Math.toRadians(position.defaultArmDegrees));
  }

  private static void check(Position position, double[][] expected, List<Transform3d> zeroed) {
    var poses = poses(position);
    for (int i = 0; i < 3; i++) {
      var rendered = poses[i].transformBy(zeroed.get(i));
      String label = position + " component " + i;
      assertEquals(expected[i][0], rendered.getX(), 0.005, label + " x");
      assertEquals(0.0, rendered.getY(), 1e-9, label + " y");
      assertEquals(expected[i][1], rendered.getZ(), 0.005, label + " z");
      assertEquals(
          expected[i][2], Math.toDegrees(rendered.getRotation().getY()), 0.1, label + " pitch");
    }
  }

  /** Reads each component's zeroed transform. Only y-axis rotations are used by this robot. */
  private static List<Transform3d> zeroedTransforms() throws IOException {
    String config = Files.readString(Path.of("advantagescope/Robot_YachenDesign/config.json"));
    var component =
        Pattern.compile(
            "\\{\\s*\"zeroedRotations\":\\s*\\[(.*?)\\],\\s*\"zeroedPosition\":\\s*\\[(.*?)\\]\\s*\\}",
            Pattern.DOTALL);
    var yRotation = Pattern.compile("\"axis\":\\s*\"y\",\\s*\"degrees\":\\s*([-0-9.]+)");
    var result = new ArrayList<Transform3d>();
    var matcher = component.matcher(config);
    while (matcher.find()) {
      String rotations = matcher.group(1).trim();
      double pitchDeg = 0.0;
      if (!rotations.isEmpty()) {
        var rotation = yRotation.matcher(rotations);
        assertTrue(rotation.find(), "Unsupported rotation: " + rotations);
        pitchDeg = Double.parseDouble(rotation.group(1));
      }
      String[] xyz = matcher.group(2).split(",");
      result.add(
          new Transform3d(
              Double.parseDouble(xyz[0].trim()),
              Double.parseDouble(xyz[1].trim()),
              Double.parseDouble(xyz[2].trim()),
              new Rotation3d(0.0, Math.toRadians(pitchDeg), 0.0)));
    }
    return result;
  }
}

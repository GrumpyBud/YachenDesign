package first.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import bline.field.FlippingUtil;
import first.robot.generated.TunerConstants;
import first.robot.sim.SimGamePieces;
import first.robot.subsystems.Choreographer;
import first.robot.subsystems.Choreographer.Position;
import first.robot.subsystems.arm.Arm;
import first.robot.subsystems.arm.ArmIOSim;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.claw.ClawIOSim;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.ModuleIOSim;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.elevator.ElevatorIOSim;
import first.robot.util.FullSubsystem;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.system.Timer;

/**
 * Runs autonomous routines against the simulated swerve and BLine. This checks sequencing, pose
 * reset, completion, and fault reporting with the ideal module simulation. It says nothing about
 * real carpet, wheel slip, or tuned gains.
 */
class AutoRoutinesSimTest {
  private static Drive drive;
  private static Elevator elevator;
  private static Arm arm;
  private static Choreographer robot;
  private static SimGamePieces pieces;
  private static List<FullSubsystem> subsystems;
  private static AutoRoutines autos;

  /** Observations from the last run, sampled every loop. */
  private static double maxElevatorMeters;

  private static double maxElevatorWhileFast;
  private static int ejectLoops;
  private static int intakeLoops;

  /** Poses below are converted to blue-alliance coordinates, so one set of checks covers red. */
  private static boolean red;

  private static double minCornerClearance;
  private static double minWallClearance;
  private static double minCenterX;
  private static Pose2d firstEjectPose;
  private static Pose2d lastEjectPose;
  private static Pose2d lastIntakePose;

  /** Above this speed the mechanisms must be retracted; the robot is tippy. */
  private static final double FAST_METERS_PER_SEC = 1.2;

  private static final double RETRACTED_ELEVATOR_METERS = 0.3;
  private static final Scheduler scheduler = Scheduler.getDefault();

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize());
    DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    SimHooks.pauseTiming();
    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
    elevator = new Elevator(new ElevatorIOSim());
    arm = new Arm(new ArmIOSim());
    pieces = new SimGamePieces(drive::getSimPose, elevator::getPositionMeters, arm::getPositionRad);
    var claw = new Claw(new ClawIOSim(pieces));
    robot = new Choreographer(drive, elevator, arm, claw);
    subsystems = List.of(drive, elevator, arm, claw);
    autos = new AutoRoutines(drive, robot);
    autos.updateInputs();
  }

  /** Runs one routine through the same loop order as RobotContainer, in autonomous. */
  private static String runToEnd(AutoRoutine routine) {
    return runToEnd(routine, () -> {});
  }

  /** As {@link #runToEnd(AutoRoutine)}, changing the simulated field after it resets. */
  private static String runToEnd(AutoRoutine routine, Runnable fieldSetup) {
    pieces.reset();
    fieldSetup.run();
    maxElevatorMeters = 0;
    maxElevatorWhileFast = 0;
    ejectLoops = 0;
    intakeLoops = 0;
    minCornerClearance = Double.POSITIVE_INFINITY;
    minWallClearance = Double.POSITIVE_INFINITY;
    minCenterX = Double.POSITIVE_INFINITY;
    firstEjectPose = null;
    lastEjectPose = null;
    lastIntakePose = null;
    Command command = autos.run(routine);
    loop();
    assertTrue(scheduler.schedule(command).successful(), routine.name());
    for (int loop = 0; loop < 50 * 30 && scheduler.isScheduledOrRunning(command); loop++) {
      loop();
    }
    assertTrue(!scheduler.isScheduledOrRunning(command), routine.name() + " never ended");
    return autos.lastStatus();
  }

  private static void loop() {
    subsystems.forEach(FullSubsystem::periodic);
    robot.periodic(true, false, false, true);
    // Applies the BLine gains. "false" keeps the chooser from starting its own routine.
    autos.periodic(false);
    scheduler.run();
    robot.applyOutputs();
    subsystems.forEach(FullSubsystem::periodicAfterScheduler);
    maxElevatorMeters = Math.max(maxElevatorMeters, elevator.getPositionMeters());
    var velocity = drive.getChassisVelocities();
    if (Math.hypot(velocity.vx, velocity.vy) > FAST_METERS_PER_SEC) {
      maxElevatorWhileFast = Math.max(maxElevatorWhileFast, elevator.getPositionMeters());
    }
    Pose2d pose = red ? FlippingUtil.flipFieldPose(drive.getPose()) : drive.getPose();
    switch (robot.getClawRequest()) {
      case EJECT -> {
        ejectLoops++;
        if (firstEjectPose == null) firstEjectPose = pose;
        lastEjectPose = pose;
      }
      case INTAKE -> {
        intakeLoops++;
        lastIntakePose = pose;
      }
      case STOP -> {}
    }
    // Bumper corners (0.44 m half-length) against the blue charge station footprint and the field
    // walls. Clearance is 0 when a corner is inside the station or past a wall. Measured from the
    // first score on, after the routine has set its starting pose.
    for (double sx : firstEjectPose == null ? new double[0] : new double[] {-0.44, 0.44}) {
      for (double sy : new double[] {-0.44, 0.44}) {
        var corner =
            pose.getTranslation().plus(new Translation2d(sx, sy).rotateBy(pose.getRotation()));
        double dx =
            Math.max(
                0,
                Math.max(
                    ChargedUpField.CHARGE_STATION_MIN_X - corner.getX(),
                    corner.getX() - ChargedUpField.CHARGE_STATION_MAX_X));
        double dy =
            Math.max(
                0,
                Math.max(
                    ChargedUpField.CHARGE_STATION_MIN_Y - corner.getY(),
                    corner.getY() - ChargedUpField.CHARGE_STATION_MAX_Y));
        minCornerClearance = Math.min(minCornerClearance, Math.hypot(dx, dy));
        minWallClearance =
            Math.min(
                minWallClearance,
                Math.min(corner.getY(), ChargedUpField.FIELD_WIDTH - corner.getY()));
      }
    }
    if (firstEjectPose != null) {
      minCenterX = Math.min(minCenterX, pose.getX());
    }
    SimHooks.stepTiming(0.02);
  }

  private static AutoRoutine routine(String name) {
    return autos.routines().stream()
        .filter(r -> r.name().equals(name))
        .findFirst()
        .orElseThrow(() -> new AssertionError("No routine " + name));
  }

  private static void onRed(Runnable test) {
    DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
    DriverStationSim.notifyNewData();
    autos.updateInputs();
    red = true;
    try {
      test.run();
    } finally {
      red = false;
      DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
      DriverStationSim.notifyNewData();
      autos.updateInputs();
    }
  }

  @Test
  void everyRoutineCompletesInSimulation() {
    for (var routine : autos.routines()) {
      String status = runToEnd(routine);
      Pose2d pose = drive.getPose();
      System.out.printf(
          "%s -> %s, final pose (%.3f, %.3f, %.1f deg)%n",
          routine.name(), status, pose.getX(), pose.getY(), pose.getRotation().getDegrees());
      assertEquals("Completed", status, routine.name());
    }
  }

  @Test
  void loadingSideOnBlue() {
    checkPlan("2023: Node 8, Piece 4, Node 7", 8, 4, 7, () -> {});
  }

  @Test
  void loadingSideOnRed() {
    onRed(() -> checkPlan("2023: Node 8, Piece 4, Node 7", 8, 4, 7, () -> {}));
  }

  @Test
  void loadingSideFallsBackWhenItsPieceIsMissing() {
    checkPlan(
        "2023: Node 8, Piece 4 (fallback 3), Node 7",
        8,
        3,
        7,
        () ->
            pieces.removeStagedNear(ChargedUpField.STAGED_PIECE_X, ChargedUpField.stagedPieceY(4)));
  }

  @Test
  void bumpSideUsesTheLowerLane() {
    checkPlan("2023: Node 2, Piece 1 (fallback 2), Node 3", 2, 1, 3, () -> {});
  }

  @Test
  void bumpSideFallsBackOnRed() {
    onRed(
        () ->
            checkPlan(
                "2023: Node 2, Piece 1 (fallback 2), Node 3",
                2,
                2,
                3,
                () ->
                    pieces.removeStagedNear(
                        // Red's piece 1, in field coordinates.
                        FlippingUtil.flipFieldPosition(
                                new Translation2d(
                                    ChargedUpField.STAGED_PIECE_X, ChargedUpField.stagedPieceY(1)))
                            .getX(),
                        ChargedUpField.stagedPieceY(1))));
  }

  /**
   * Runs a scoring routine and checks it in blue-alliance coordinates.
   *
   * @param pickedPiece the staged piece expected in the claw, after any fallback
   */
  private static void checkPlan(
      String name, int startNode, int pickedPiece, int scoreNode, Runnable fieldSetup) {
    double start = Timer.getTimestamp();
    String status = runToEnd(routine(name), fieldSetup);
    double elapsed = Timer.getTimestamp() - start;
    String label = name + (red ? " (red)" : " (blue)");
    System.out.printf(
        "%s -> %s in %.1f s; preload at (%.3f, %.3f); claw ended at (%.3f, %.3f); second score"
            + " at (%.3f, %.3f, %.1f deg); closest corner to charge station %.3f m, to a wall"
            + " %.3f m; closest center to grid x %.3f; max elevator %.3f m, %.3f m while moving"
            + " fast%n",
        label,
        status,
        elapsed,
        firstEjectPose == null ? Double.NaN : firstEjectPose.getX(),
        firstEjectPose == null ? Double.NaN : firstEjectPose.getY(),
        lastIntakePose == null ? Double.NaN : lastIntakePose.getX() + 0.586,
        lastIntakePose == null ? Double.NaN : lastIntakePose.getY(),
        lastEjectPose == null ? Double.NaN : lastEjectPose.getX(),
        lastEjectPose == null ? Double.NaN : lastEjectPose.getY(),
        lastEjectPose == null ? Double.NaN : lastEjectPose.getRotation().getDegrees(),
        minCornerClearance,
        minWallClearance,
        minCenterX,
        maxElevatorMeters,
        maxElevatorWhileFast);
    assertEquals("Completed", status, label);
    assertTrue(elapsed < 15.0, label + " fits in autonomous: " + elapsed);
    // Both scores against the grid at the chosen nodes, facing it.
    assertEquals(ChargedUpField.GRID_X, firstEjectPose.getX(), 0.03, label);
    assertEquals(ChargedUpField.nodeY(startNode), firstEjectPose.getY(), 0.03, label);
    assertEquals(ChargedUpField.GRID_X, lastEjectPose.getX(), 0.06, label);
    assertEquals(ChargedUpField.nodeY(scoreNode), lastEjectPose.getY(), 0.06, label);
    assertEquals(180, Math.abs(lastEjectPose.getRotation().getDegrees()), 3, label);
    // The claw ends centered on the expected piece, slightly past it.
    double clawX = lastIntakePose.getX() + ChargedUpField.CLAW_REACH_AT_INTAKE;
    assertTrue(
        clawX >= ChargedUpField.STAGED_PIECE_X && clawX <= ChargedUpField.STAGED_PIECE_X + 0.15,
        label + " claw ended at x " + clawX + ", not on the piece");
    assertEquals(ChargedUpField.stagedPieceY(pickedPiece), lastIntakePose.getY(), 0.06, label);
    // Bumpers never enter the charge station, leave the field, or push into the grid.
    assertTrue(minCornerClearance > 0.05, label + " charge station: " + minCornerClearance);
    assertTrue(minWallClearance > 0.03, label + " wall: " + minWallClearance);
    assertTrue(minCenterX > ChargedUpField.GRID_X - 0.06, label + " grid: " + minCenterX);
    // Tippy robot: retracted whenever it is moving fast, extended only near a stop.
    assertTrue(maxElevatorMeters > 1.28, label + " reached the scoring height");
    assertTrue(
        maxElevatorWhileFast < RETRACTED_ELEVATOR_METERS,
        label + " extended while moving fast: " + maxElevatorWhileFast);
    assertEquals(30, ejectLoops, 2, label + " two ejects");
    assertTrue(intakeLoops > 25, label + " intake ran: " + intakeLoops);
    assertEquals(Position.STOW, robot.getPosition(), label);
    for (int i = 0; i < 50 && !robot.atPosition(); i++) {
      loop();
    }
    assertTrue(robot.atPosition(), label + " stowed within 1 s after the routine");
    assertEquals(Claw.Request.STOP, robot.getClawRequest(), label);
    // The simulated field saw the preload and the picked-up piece leave the claw.
    assertEquals(2, pieces.releasedCount(), label + " pieces scored");
  }

  @Test
  void missingEveryPieceStopsTheRoutine() {
    String status =
        runToEnd(routine("2023: Node 8, Piece 4 (fallback 3), Node 7"), pieces::clearStaged);
    assertTrue(status.startsWith("No game piece detected after trying pieces [4, 3]"), status);
    // Only the preload left the claw, the rollers stopped, and the joints retracted.
    assertEquals(1, pieces.releasedCount());
    assertEquals(Claw.Request.STOP, robot.getClawRequest());
    assertEquals(Position.STOW, robot.getPosition());
  }

  @Test
  void onlyTheOwnerCanMoveMechanisms() {
    loop(); // Autonomous owns the mechanisms in this test fixture.
    assertTrue(robot.requestPosition(Choreographer.Source.AUTO, Position.INTAKE));
    assertTrue(!robot.requestPosition(Choreographer.Source.DRIVER, Position.SCORE));
    assertTrue(!robot.requestClaw(Choreographer.Source.DRIVER, Claw.Request.INTAKE));
    assertEquals(Position.INTAKE, robot.getPosition());
    assertTrue(robot.requestPosition(Choreographer.Source.AUTO, Position.STOW));
  }

  @Test
  void outAndBackEndsAtItsStart() {
    String status =
        runToEnd(
            new AutoRoutine(
                "Out and Back Check",
                12,
                auto -> {
                  if (!auto.follow(AutoPaths.drive1Meter())) return;
                  auto.follow(AutoPaths.driveBack1Meter());
                }));
    assertEquals("Completed", status);
    assertEquals(2.0, drive.getPose().getX(), 0.08);
    assertEquals(4.0, drive.getPose().getY(), 0.08);
  }

  @Test
  void redAllianceTransformsTheJavaPathOnce() {
    DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
    DriverStationSim.notifyNewData();
    try {
      autos.updateInputs();
      String status = runToEnd(AutoRoutine.singlePath("Red", 5, AutoPaths::drive1Meter));
      assertEquals("Completed", status);
      // Blue end pose (3, 4, 0 deg), transformed with the field's configured symmetry.
      Pose2d expected = FlippingUtil.flipFieldPose(new Pose2d(3, 4, Rotation2d.ZERO));
      Pose2d actual = drive.getPose();
      assertEquals(expected.getX(), actual.getX(), 0.08);
      assertEquals(expected.getY(), actual.getY(), 0.08);
      assertEquals(
          0, actual.getRotation().minus(expected.getRotation()).getDegrees(), 3, "heading");
    } finally {
      DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
      DriverStationSim.notifyNewData();
      autos.updateInputs();
    }
  }

  @Test
  void routineTimeoutIsReportedAsFailure() {
    String status = runToEnd(AutoRoutine.singlePath("Too Short", 0.5, AutoPaths::drive1Meter));
    assertTrue(status.startsWith("Timed out"), status);
  }

  @Test
  void segmentTimeoutStopsLaterSteps() {
    boolean[] reachedLaterStep = {false};
    String status =
        runToEnd(
            new AutoRoutine(
                "Segment Timeout",
                10,
                auto -> {
                  if (!auto.follow(AutoPaths.drive1Meter(), 0.3)) return;
                  reachedLaterStep[0] = true;
                }));
    assertTrue(status.startsWith("Path did not reach its endpoint"), status);
    assertTrue(!reachedLaterStep[0]);
  }

  @Test
  void missingDrawnPathIsReported() {
    String status =
        runToEnd(new AutoRoutine("Missing Drawn", 5, auto -> auto.followDrawn("does-not-exist")));
    assertTrue(status.startsWith("Could not load drawn path does-not-exist"), status);
  }
}

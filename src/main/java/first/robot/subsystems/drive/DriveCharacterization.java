package first.robot.subsystems.drive;

import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/** Hold-to-run versions of AdvantageKit's wheel radius and simple voltage feedforward tests. */
public final class DriveCharacterization {
  private static final String LOG = "Drive/Characterization/";
  private static final double STEER_SETTLE_SECONDS = 0.25;
  private static final double STEER_TIMEOUT_SECONDS = 2.0;
  private static final double STEER_TOLERANCE_COS = Math.cos(Math.toRadians(10));
  private static final double FF_RAMP_VOLTS_PER_SECOND = 0.4;
  private static final double FF_MAX_VOLTS = 2.5;
  private static final double FF_TIMEOUT_SECONDS = 10.0;
  private static final double WHEEL_MAX_RAD_PER_SEC = 0.8;
  private static final double WHEEL_RAMP_RAD_PER_SEC2 = 0.5;
  private static final double WHEEL_TIMEOUT_SECONDS = 18.0;
  private static final double FULL_TURN_RADIANS = 2 * Math.PI;

  private enum Test {
    WHEEL_RADIUS(WHEEL_TIMEOUT_SECONDS),
    DRIVE_FEEDFORWARD(FF_TIMEOUT_SECONDS);

    final double timeoutSeconds;

    Test(double timeoutSeconds) {
      this.timeoutSeconds = timeoutSeconds;
    }
  }

  record ModuleSample(
      boolean connected,
      Rotation2d angle,
      double wheelPositionRad,
      double wheelSpeedRps,
      double appliedVolts) {
    boolean valid() {
      return connected
          && Double.isFinite(angle.getRadians())
          && Double.isFinite(wheelPositionRad)
          && Double.isFinite(wheelSpeedRps)
          && Double.isFinite(appliedVolts);
    }
  }

  private final Drive drive;
  private final Alert fault =
      new Alert("DriveCharacterization", "Drive characterization stopped.", Level.MEDIUM);
  private boolean armed;
  private Test active;
  private double startedAt;
  private double alignedAt;
  private double measuringAt;
  private Rotation2d lastYaw;
  private double yawTravelRad;
  private double[] startingWheelRadians;
  private VoltageFit[] fits;

  public DriveCharacterization(Drive drive) {
    this.drive = drive;
  }

  /** Runs after the scheduler, before drivetrain outputs are applied. */
  public void periodic(boolean allowed, boolean wheelHeld, boolean feedforwardHeld) {
    if (!allowed) {
      armed = false;
      if (active != null) {
        end("Stopped: Utility mode ended or controller lost", false);
      }
      // The new owner may already have requested drive output this cycle.
      return;
    }

    if (active == null) {
      drive.neutral();
      if (!wheelHeld && !feedforwardHeld) {
        armed = true;
        return;
      }
      if (!armed) {
        return;
      }
      armed = false;
      if (wheelHeld && feedforwardHeld) {
        Logger.recordOutput(LOG + "ResultValid", false);
        end("Hold only one characterization button", true);
        return;
      }
      begin(wheelHeld ? Test.WHEEL_RADIUS : Test.DRIVE_FEEDFORWARD);
    }

    if (wheelHeld && feedforwardHeld) {
      stop("Stopped: both characterization buttons held", true);
      return;
    }
    if (active == Test.WHEEL_RADIUS ? !wheelHeld : !feedforwardHeld) {
      stop("Stopped: button released before completion", false);
      return;
    }

    double now = Timer.getTimestamp();
    if (now - startedAt >= active.timeoutSeconds) {
      stop("Timed out before enough data was collected", true);
      return;
    }
    var modules = drive.getCharacterizationSamples();
    if (Arrays.stream(modules).anyMatch(sample -> !sample.valid())) {
      stop("A swerve motor or steering encoder is unavailable", true);
      return;
    }
    if (active == Test.WHEEL_RADIUS && !drive.hasCharacterizationGyro()) {
      stop("Wheel radius requires an independent connected gyro", true);
      return;
    }
    if (active == Test.DRIVE_FEEDFORWARD && !drive.usesDriveVoltageControl()) {
      stop("Drive feedforward requires voltage control on every module", true);
      return;
    }

    if (Double.isNaN(measuringAt)) {
      steerAndSetBaseline(now, modules);
    } else if (!modulesAligned(modules)) {
      stop("A module left its required steering angle", true);
    } else if (active == Test.WHEEL_RADIUS) {
      measureWheelRadius(now - measuringAt, modules);
    } else {
      measureFeedforward(now - measuringAt, modules);
    }
  }

  private void begin(Test test) {
    active = test;
    startedAt = Timer.getTimestamp();
    alignedAt = Double.NaN;
    measuringAt = Double.NaN;
    fault.set(false);
    Logger.recordOutput(LOG + "LastTest", test);
    Logger.recordOutput(LOG + "Active", true);
    Logger.recordOutput(LOG + "ResultValid", false);
    Logger.recordOutput(LOG + "Status", "Aligning modules");
  }

  private void steerAndSetBaseline(double now, ModuleSample[] modules) {
    if (active == Test.WHEEL_RADIUS) {
      drive.orientForWheelRadiusTest();
    } else {
      drive.requestCharacterizationVoltage(0);
    }
    if (!modulesAligned(modules)) {
      alignedAt = Double.NaN;
    } else if (Double.isNaN(alignedAt)) {
      alignedAt = now;
    }

    if (now - alignedAt >= STEER_SETTLE_SECONDS) {
      measuringAt = now;
      if (active == Test.WHEEL_RADIUS) {
        lastYaw = drive.getCharacterizationYaw();
        yawTravelRad = 0;
        startingWheelRadians =
            Arrays.stream(modules).mapToDouble(ModuleSample::wheelPositionRad).toArray();
      } else {
        fits = new VoltageFit[modules.length];
        Arrays.setAll(fits, ignored -> new VoltageFit());
      }
      Logger.recordOutput(LOG + "Status", "Measuring");
    } else if (now - startedAt >= STEER_TIMEOUT_SECONDS) {
      stop("Modules could not align before the steering timeout", true);
    }
  }

  private boolean modulesAligned(ModuleSample[] modules) {
    var locations = Drive.getModuleTranslations();
    for (int i = 0; i < modules.length; i++) {
      double alignment = modules[i].angle().getCos();
      if (active == Test.WHEEL_RADIUS) {
        var tangent = locations[i].getAngle().orElseThrow().plus(Rotation2d.CCW_PI_2);
        // A module may steer to the opposite tangent and reverse its wheel.
        alignment = Math.abs(modules[i].angle().minus(tangent).getCos());
      }
      if (alignment < STEER_TOLERANCE_COS) {
        return false;
      }
    }
    return true;
  }

  private void measureWheelRadius(double elapsed, ModuleSample[] modules) {
    var yaw = drive.getCharacterizationYaw();
    yawTravelRad += yaw.minus(lastYaw).getRadians();
    lastYaw = yaw;
    double yawMagnitude = Math.abs(yawTravelRad);
    Logger.recordOutput(LOG + "WheelRadius/YawRadians", yawTravelRad);
    if (yawMagnitude < FULL_TURN_RADIANS) {
      double speed = Math.min(WHEEL_MAX_RAD_PER_SEC, WHEEL_RAMP_RAD_PER_SEC2 * elapsed);
      if (speed > 0) {
        drive.runVelocity(new ChassisVelocities(0, 0, speed));
      }
      return;
    }

    var locations = Drive.getModuleTranslations();
    var wheelTravelRad = new double[modules.length];
    var radii = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      wheelTravelRad[i] = Math.abs(modules[i].wheelPositionRad() - startingWheelRadians[i]);
      if (wheelTravelRad[i] < 0.5) {
        stop("Wheel radius requires encoder travel from all four wheels", true);
        return;
      }
      radii[i] = yawMagnitude * locations[i].getNorm() / wheelTravelRad[i];
    }
    Logger.recordOutput(LOG + "WheelRadius/WheelTravelRadians", wheelTravelRad);
    Logger.recordOutput(LOG + "WheelRadius/PerModuleMeters", radii);
    Logger.recordOutput(
        LOG + "WheelRadius/MeanMeters", Arrays.stream(radii).average().orElseThrow());
    complete();
  }

  private void measureFeedforward(double elapsed, ModuleSample[] modules) {
    for (int i = 0; i < modules.length; i++) {
      var sample = modules[i];
      if (sample.wheelSpeedRps() >= 0.1 && sample.appliedVolts() > 0) {
        fits[i].add(sample.wheelSpeedRps(), sample.appliedVolts());
      }
    }

    double requestedVolts = elapsed * FF_RAMP_VOLTS_PER_SECOND;
    if (requestedVolts < FF_MAX_VOLTS) {
      drive.requestCharacterizationVoltage(requestedVolts);
      Logger.recordOutput(LOG + "Feedforward/RequestedVolts", requestedVolts);
      return;
    }

    var kS = new double[fits.length];
    var kV = new double[fits.length];
    var counts = new int[fits.length];
    var speedRanges = new double[fits.length];
    for (int i = 0; i < fits.length; i++) {
      var fit = fits[i];
      counts[i] = fit.count;
      speedRanges[i] = fit.speedRange();
    }
    Logger.recordOutput(LOG + "Feedforward/SamplesPerModule", counts);
    Logger.recordOutput(LOG + "Feedforward/SpeedRangeRps", speedRanges);
    for (int i = 0; i < fits.length; i++) {
      var fit = fits[i];
      if (fit.count < 20 || fit.speedRange() < 0.5 || fit.speedVariance <= 0) {
        stop(
            "Feedforward needs more moving samples across a wider speed range on module " + i,
            true);
        return;
      }
      kV[i] = fit.voltageCovariance / fit.speedVariance;
      kS[i] = fit.meanVoltage - kV[i] * fit.meanSpeed;
      if (!Double.isFinite(kS[i]) || !Double.isFinite(kV[i]) || kV[i] <= 0) {
        stop("Feedforward fit was invalid on module " + i, true);
        return;
      }
    }
    Logger.recordOutput(LOG + "Feedforward/ModuleKsVolts", kS);
    Logger.recordOutput(LOG + "Feedforward/ModuleKvVoltsPerRps", kV);
    Logger.recordOutput(LOG + "Feedforward/MeanKsVolts", Arrays.stream(kS).average().orElseThrow());
    Logger.recordOutput(
        LOG + "Feedforward/MeanKvVoltsPerRps", Arrays.stream(kV).average().orElseThrow());
    complete();
  }

  private void complete() {
    Logger.recordOutput(LOG + "ResultValid", true);
    stop("Complete", false);
  }

  private void stop(String reason, boolean showAlert) {
    drive.neutral();
    end(reason, showAlert);
  }

  private void end(String reason, boolean showAlert) {
    active = null;
    Logger.recordOutput(LOG + "Active", false);
    Logger.recordOutput(LOG + "Feedforward/RequestedVolts", 0.0);
    Logger.recordOutput(LOG + "Status", reason);
    fault.setText(reason);
    fault.set(showAlert);
  }

  /** Streaming least-squares fit of applied voltage against measured wheel rotations per second. */
  private static final class VoltageFit {
    int count;
    double meanSpeed;
    double meanVoltage;
    double speedVariance;
    double voltageCovariance;
    double minimumSpeed = Double.POSITIVE_INFINITY;
    double maximumSpeed = Double.NEGATIVE_INFINITY;

    void add(double speed, double voltage) {
      count++;
      double speedDelta = speed - meanSpeed;
      meanSpeed += speedDelta / count;
      meanVoltage += (voltage - meanVoltage) / count;
      speedVariance += speedDelta * (speed - meanSpeed);
      voltageCovariance += speedDelta * (voltage - meanVoltage);
      minimumSpeed = Math.min(minimumSpeed, speed);
      maximumSpeed = Math.max(maximumSpeed, speed);
    }

    double speedRange() {
      return count == 0 ? 0 : maximumSpeed - minimumSpeed;
    }
  }
}

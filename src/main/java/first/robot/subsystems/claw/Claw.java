package first.robot.subsystems.claw;

import first.robot.util.FullSubsystem;
import first.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/** Roller claw. Runs only in the loops where a caller requests intake or eject. */
public class Claw extends FullSubsystem {
  public enum Request {
    STOP,
    INTAKE,
    EJECT
  }

  private static final double HOT_MOTOR_CELSIUS = 70.0;
  // A roller drawing this much while barely turning is stalled, e.g. holding a game piece.
  private static final double STALL_CURRENT_AMPS = 30.0;
  private static final double STALL_VELOCITY_ROT_PER_SEC = 2.0;

  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
  private final Alert disconnectedAlert =
      new Alert("ClawDisconnected", "Claw motor disconnected.", Level.MEDIUM);
  private final Alert hotAlert = new Alert("ClawHot", "Claw motor above 70 C.", Level.MEDIUM);
  private final LoggedTunableNumber intakeVolts = new LoggedTunableNumber("Claw/IntakeVolts", 8.0);
  private final LoggedTunableNumber ejectVolts = new LoggedTunableNumber("Claw/EjectVolts", -6.0);
  private Request request = Request.STOP;

  public Claw(ClawIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);
    disconnectedAlert.set(!inputs.connected);
    hotAlert.set(inputs.tempCelsius > HOT_MOTOR_CELSIUS);
  }

  /** Request roller output for this loop only. */
  public void request(Request request) {
    this.request = request;
  }

  @Override
  public void periodicAfterScheduler() {
    double volts =
        switch (request) {
          case STOP -> 0.0;
          case INTAKE -> intakeVolts.get();
          case EJECT -> ejectVolts.get();
        };
    io.setVoltage(volts);
    Logger.recordOutput("Claw/Request", request);
    Logger.recordOutput("Claw/RequestedVolts", volts);
    Logger.recordOutput(
        "Diagnostics/Claw/Stalled",
        volts != 0.0
            && inputs.statorCurrentAmps > STALL_CURRENT_AMPS
            && Math.abs(inputs.velocityRotPerSec) < STALL_VELOCITY_ROT_PER_SEC);
    Logger.recordOutput("Diagnostics/Claw/StatorCurrentAmps", inputs.statorCurrentAmps);
    request = Request.STOP;
  }
}

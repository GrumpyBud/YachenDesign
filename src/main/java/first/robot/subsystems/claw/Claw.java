package first.robot.subsystems.claw;

import first.robot.util.FullSubsystem;
import first.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Roller claw. Runs only in the loops where a caller requests intake or eject.
 *
 * <p>Game piece detection uses motor torque (stator current): while intaking, a piece jams the
 * rollers, so current stays high while roller speed drops. Spin-up also draws high current at low
 * speed, so the first moments after intake starts are ignored. Ejecting long enough clears the
 * piece. Detection uses only logged inputs, so it replays.
 */
public class Claw extends FullSubsystem {
  public enum Request {
    STOP,
    INTAKE,
    EJECT
  }

  private static final double HOT_MOTOR_CELSIUS = 70.0;

  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
  private final Alert disconnectedAlert =
      new Alert("ClawDisconnected", "Claw motor disconnected.", Level.MEDIUM);
  private final Alert hotAlert = new Alert("ClawHot", "Claw motor above 70 C.", Level.MEDIUM);
  private final LoggedTunableNumber intakeVolts = new LoggedTunableNumber("Claw/IntakeVolts", 8.0);
  private final LoggedTunableNumber ejectVolts = new LoggedTunableNumber("Claw/EjectVolts", -6.0);

  // Detection thresholds. Estimates until measured with real game pieces.
  private final LoggedTunableNumber pieceCurrentAmps =
      new LoggedTunableNumber("Claw/PieceCurrentAmps", 30.0);
  private final LoggedTunableNumber pieceMaxVelocity =
      new LoggedTunableNumber("Claw/PieceMaxVelocityRotPerSec", 2.0);
  private final LoggedTunableNumber spinUpIgnoreSeconds =
      new LoggedTunableNumber("Claw/SpinUpIgnoreSeconds", 0.25);
  private final LoggedTunableNumber pieceConfirmSeconds =
      new LoggedTunableNumber("Claw/PieceConfirmSeconds", 0.1);
  private final LoggedTunableNumber ejectClearSeconds =
      new LoggedTunableNumber("Claw/EjectClearSeconds", 0.25);

  private Request request = Request.STOP;
  private Request applied = Request.STOP;
  private double appliedSince = 0.0;
  private double stalledSince = Double.NaN;
  private boolean hasGamePiece = false;

  public Claw(ClawIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);
    disconnectedAlert.set(!inputs.connected);
    hotAlert.set(inputs.tempCelsius > HOT_MOTOR_CELSIUS);
    updateDetection();
  }

  /** Inputs reflect last loop's output, so detection judges the request that was applied. */
  private void updateDetection() {
    double now = Timer.getTimestamp();
    boolean stalled =
        inputs.connected
            && inputs.statorCurrentAmps > pieceCurrentAmps.get()
            && Math.abs(inputs.velocityRotPerSec) < pieceMaxVelocity.get();
    switch (applied) {
      case INTAKE -> {
        if (stalled && now - appliedSince >= spinUpIgnoreSeconds.get()) {
          if (Double.isNaN(stalledSince)) {
            stalledSince = now;
          }
          if (now - stalledSince >= pieceConfirmSeconds.get()) {
            hasGamePiece = true;
          }
        } else {
          stalledSince = Double.NaN;
        }
      }
      case EJECT -> {
        stalledSince = Double.NaN;
        if (now - appliedSince >= ejectClearSeconds.get()) {
          hasGamePiece = false;
        }
      }
      case STOP -> stalledSince = Double.NaN;
    }
    Logger.recordOutput("Claw/HasGamePiece", hasGamePiece);
    Logger.recordOutput("Diagnostics/Claw/Stalled", stalled);
  }

  /** Whether a game piece has been detected and not ejected since. */
  public boolean hasGamePiece() {
    return hasGamePiece;
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
    if (request != applied) {
      applied = request;
      appliedSince = Timer.getTimestamp();
    }
    Logger.recordOutput("Claw/Request", request);
    Logger.recordOutput("Claw/RequestedVolts", volts);
    Logger.recordOutput("Diagnostics/Claw/StatorCurrentAmps", inputs.statorCurrentAmps);
    request = Request.STOP;
  }
}

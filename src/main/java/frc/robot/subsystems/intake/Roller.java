// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Roller {
  // Tunable numbers
  public static final LoggedTunableNumber intakeVelocity =
      new LoggedTunableNumber("Intake/Roller/IntakeVelocityRPS", 60.0);
  public static final LoggedTunableNumber ejectVelocity =
      new LoggedTunableNumber("Intake/Roller/EjectVelocityRPS", -40.0);
  public static final LoggedTunableNumber holdVelocity =
      new LoggedTunableNumber("Intake/Roller/HoldVelocityRPS", 5.0);
  private static final LoggedTunableNumber currentThreshold =
      new LoggedTunableNumber("Intake/Roller/CurrentThreshold", 13.0);

  // Hardware
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  // Game piece detection
  @Getter
  @AutoLogOutput(key = "Intake/Roller/HasGamePiece")
  private boolean hasGamePiece = false;

  private final Debouncer gamePieceDebouncer = new Debouncer(0.1);

  // Disconnected alerts
  private final Alert motorDisconnectedAlert =
      new Alert("Intake roller motor disconnected!", Alert.AlertType.kWarning);
  private final Alert canRangeDisconnectedAlert =
      new Alert("Intake CANRange disconnected!", Alert.AlertType.kWarning);

  public Roller(RollerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Roller", inputs);

    motorDisconnectedAlert.set(!inputs.talonConnected && Constants.getCurrentMode() == Mode.REAL);
    canRangeDisconnectedAlert.set(
        !inputs.CANRangeConnected && Constants.getCurrentMode() == Mode.REAL);

    // Update game piece detection from CANRange
    if (Constants.getCurrentMode() != Mode.SIM) {
      hasGamePiece = inputs.hasCoral;
    }
  }

  /** Run roller at velocity (rotations per second) using torque current control */
  public void runVelocity(double velocityRPS) {
    io.runVelocity(velocityRPS);
  }

  /** Run roller at intake velocity */
  public void runIntake() {
    runVelocity(intakeVelocity.get());
  }

  /** Run roller at eject velocity */
  public void runEject() {
    runVelocity(ejectVelocity.get());
  }

  /** Run roller at hold velocity */
  public void runHold() {
    runVelocity(holdVelocity.get());
  }

  /** Run roller at voltage */
  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  /** Run roller at torque current */
  public void runTorqueCurrent(double current) {
    io.runTorqueCurrent(current);
  }

  /** Stop the roller */
  public void stop() {
    io.stop();
  }

  /** Set brake mode */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  /** Configure velocity PID gains */
  public void configurePID(double kP, double kI, double kD, double kS, double kV) {
    io.configurePID(kP, kI, kD, kS, kV);
  }

  /** Get current velocity in radians per second */
  public double getVelocityRadsPerSec() {
    return inputs.talonVelocityRadsPerSec;
  }

  /** Get supply current in amps */
  public double getSupplyCurrentAmps() {
    return inputs.talonSupplyCurrentAmps;
  }

  /** Check for game piece using current spike detection */
  public boolean detectGamePieceFromCurrent() {
    return gamePieceDebouncer.calculate(
        Math.abs(inputs.talonSupplyCurrentAmps) >= currentThreshold.get());
  }

  /** Set game piece state (for simulation) */
  public void setHasGamePiece(boolean hasGamePiece) {
    this.hasGamePiece = hasGamePiece;
  }
}

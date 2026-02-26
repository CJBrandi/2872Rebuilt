// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
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
  private boolean runningProfile = false;
  private double profileSetpointVelocityRPS = 0.0;
  private double profileGoalVelocityRPS = 0.0;

  // Disconnected alerts
  private final Alert motorDisconnectedAlert =
      new Alert("Intake roller motor disconnected!", Alert.AlertType.kWarning);
  private final Alert canRangeDisconnectedAlert =
      new Alert("Intake CANRange disconnected!", Alert.AlertType.kWarning);

  public Roller(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Roller", inputs);

    motorDisconnectedAlert.set(!inputs.talonConnected && Constants.getCurrentMode() == Mode.REAL);
    canRangeDisconnectedAlert.set(
        !inputs.CANRangeConnected && Constants.getCurrentMode() == Mode.REAL);

    Logger.recordOutput("Intake/Roller/MeasuredPositionRad", inputs.talonPositionRads);
    Logger.recordOutput("Intake/Roller/MeasuredVelocityRadPerSec", inputs.talonVelocityRadsPerSec);
    Logger.recordOutput(
        "Intake/Roller/MeasuredVelocityRPS",
        Units.radiansToRotations(inputs.talonVelocityRadsPerSec));
    Logger.recordOutput("Intake/Roller/RunningProfile", runningProfile);

    if (runningProfile) {
      profileSetpointVelocityRPS = profileGoalVelocityRPS;
      Logger.recordOutput("Intake/Roller/Profile/SetpointVelocityRPS", profileSetpointVelocityRPS);
      Logger.recordOutput(
          "Intake/Roller/Profile/SetpointVelocityRadPerSec",
          Units.rotationsToRadians(profileSetpointVelocityRPS));
      Logger.recordOutput("Intake/Roller/Profile/GoalVelocityRPS", profileGoalVelocityRPS);
      Logger.recordOutput(
          "Intake/Roller/Profile/GoalVelocityRadPerSec",
          Units.rotationsToRadians(profileGoalVelocityRPS));
    } else {
      Logger.recordOutput("Intake/Roller/Profile/SetpointVelocityRPS", 0.0);
      Logger.recordOutput("Intake/Roller/Profile/SetpointVelocityRadPerSec", 0.0);
      Logger.recordOutput("Intake/Roller/Profile/GoalVelocityRPS", 0.0);
      Logger.recordOutput("Intake/Roller/Profile/GoalVelocityRadPerSec", 0.0);
    }
  }

  /** Run roller at velocity (rotations per second) using torque current control */
  public void runVelocity(double velocityRPS) {
    runningProfile = true;
    profileGoalVelocityRPS = velocityRPS;
    profileSetpointVelocityRPS = velocityRPS;
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
    runningProfile = false;
    profileSetpointVelocityRPS = 0.0;
    profileGoalVelocityRPS = 0.0;
    io.runVolts(volts);
  }

  /** Run roller at torque current */
  public void runTorqueCurrent(double current) {
    runningProfile = false;
    profileSetpointVelocityRPS = 0.0;
    profileGoalVelocityRPS = 0.0;
    io.runTorqueCurrent(current);
  }

  /** Stop the roller */
  public void stop() {
    runningProfile = false;
    profileSetpointVelocityRPS = 0.0;
    profileGoalVelocityRPS = 0.0;
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
}

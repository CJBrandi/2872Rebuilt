// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
  // Manual mode tunables
  private static final LoggedTunableNumber manualModeEnabled =
      new LoggedTunableNumber("Manual/Enabled", 0.0);
  private static final LoggedTunableNumber manualRollerRPM =
      new LoggedTunableNumber("Manual/IntakeRollerRPM", 0.0);

  // Closed-loop velocity tuning (Talon velocity-loop units)
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Roller/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/Roller/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/Roller/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/Roller/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/Roller/kV");
  private static final LoggedTunableNumber maxVelocityRPS =
      new LoggedTunableNumber("Intake/Roller/MaxVelocityRPS");
  private static final LoggedTunableNumber maxAccelerationRPS2 =
      new LoggedTunableNumber("Intake/Roller/MaxAccelerationRPS2");
  private static final LoggedTunableNumber velocityToleranceRPM =
      new LoggedTunableNumber("Intake/Roller/VelocityToleranceRPM");

  // Preset targets
  public static final LoggedTunableNumber intakeVelocity =
      new LoggedTunableNumber("Intake/Roller/IntakeVelocityRPS", 60.0);
  public static final LoggedTunableNumber ejectVelocity =
      new LoggedTunableNumber("Intake/Roller/EjectVelocityRPS", -40.0);
  public static final LoggedTunableNumber holdVelocity =
      new LoggedTunableNumber("Intake/Roller/HoldVelocityRPS", 5.0);

  static {
    switch (Constants.getCurrentMode()) {
      case REAL -> {
        kP.initDefault(5);
        kI.initDefault(0.0);
        kD.initDefault(0.0);
        kS.initDefault(0.2);
        kV.initDefault(0.3);
      }
      case SIM, REPLAY -> {
        kP.initDefault(0.5);
        kI.initDefault(0.0);
        kD.initDefault(0.0);
        kS.initDefault(0.1);
        kV.initDefault(0.12);
      }
    }
    maxVelocityRPS.initDefault(120.0);
    maxAccelerationRPS2.initDefault(240.0);
    velocityToleranceRPM.initDefault(100.0);
  }

  // Hardware
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(maxVelocityRPS.get(), maxAccelerationRPS2.get()));
  @Getter private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0.0, 0.0);
  private double targetPositionRot = 0.0;
  @Getter private double targetVelocityRPS = 0.0;
  @Getter private boolean atSetpoint = false;
  private boolean closedLoop = false;
  private boolean manualOverrideActive = false;
  private boolean wasManualMode = false;

  // Disconnected alerts
  private final Alert motorDisconnectedAlert =
      new Alert("Intake roller motor disconnected!", Alert.AlertType.kWarning);

  public Roller(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Roller", inputs);

    boolean manualModeEnabledNow = manualModeEnabled.get() > 0.5;
    if (!manualModeEnabledNow) {
      manualOverrideActive = false;
    } else if (!wasManualMode || manualRollerRPM.hasChanged(hashCode())) {
      manualOverrideActive = false;
    }
    boolean manualMode = manualModeEnabledNow && !manualOverrideActive;
    if (manualMode) {
      runVelocityRPM(manualRollerRPM.get());
    } else if (!manualModeEnabledNow && wasManualMode) {
      stop();
    }
    wasManualMode = manualModeEnabledNow;

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.configurePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get()),
        kP,
        kI,
        kD,
        kS,
        kV);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            profile =
                new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                        maxVelocityRPS.get(), maxAccelerationRPS2.get())),
        maxVelocityRPS,
        maxAccelerationRPS2);

    if (closedLoop) {
      // Move a virtual position target at desired steady-state velocity and profile towards it.
      targetPositionRot += targetVelocityRPS * Constants.loopPeriodSecs;
      var goalState = new TrapezoidProfile.State(targetPositionRot, targetVelocityRPS);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      io.runVelocity(setpoint.velocity);

      atSetpoint =
          EqualsUtil.epsilonEquals(
              getMeasuredVelocityRPS(), setpoint.velocity, velocityToleranceRPM.get() / 60.0);
    } else {
      atSetpoint = false;
    }

    motorDisconnectedAlert.set(!inputs.talonConnected && Constants.getCurrentMode() == Mode.REAL);

    double measuredVelocityRPS = getMeasuredVelocityRPS();

    Logger.recordOutput("Intake/Roller/MeasuredPositionRad", inputs.talonPositionRads);
    Logger.recordOutput("Intake/Roller/MeasuredVelocityRadPerSec", inputs.talonVelocityRadsPerSec);
    Logger.recordOutput("Intake/Roller/MeasuredVelocityRPS", measuredVelocityRPS);
    Logger.recordOutput("Intake/Roller/MeasuredVelocityRPM", measuredVelocityRPS * 60.0);
    Logger.recordOutput("Intake/Roller/RunningProfile", closedLoop);
    Logger.recordOutput("Intake/Roller/ClosedLoop", closedLoop);
    Logger.recordOutput("Intake/Roller/AtSetpoint", atSetpoint);
    Logger.recordOutput("Intake/Roller/TargetVelocityRPM", targetVelocityRPS * 60.0);
    Logger.recordOutput(
        "Intake/Roller/SetpointVelocityRPM", closedLoop ? setpoint.velocity * 60.0 : 0.0);
    Logger.recordOutput("Intake/Roller/ManualMode", manualMode);
    Logger.recordOutput("Intake/Roller/ManualModeEnabled", manualModeEnabledNow);
    Logger.recordOutput("Intake/Roller/ManualOverrideActive", manualOverrideActive);
    Logger.recordOutput("Intake/Roller/Manual/TargetRPM", manualRollerRPM.get());

    if (closedLoop) {
      Logger.recordOutput("Intake/Roller/Profile/SetpointVelocityRPS", setpoint.velocity);
      Logger.recordOutput(
          "Intake/Roller/Profile/SetpointVelocityRadPerSec",
          Units.rotationsToRadians(setpoint.velocity));
      Logger.recordOutput("Intake/Roller/Profile/GoalVelocityRPS", targetVelocityRPS);
      Logger.recordOutput(
          "Intake/Roller/Profile/GoalVelocityRadPerSec",
          Units.rotationsToRadians(targetVelocityRPS));
    } else {
      Logger.recordOutput("Intake/Roller/Profile/SetpointVelocityRPS", 0.0);
      Logger.recordOutput("Intake/Roller/Profile/SetpointVelocityRadPerSec", 0.0);
      Logger.recordOutput("Intake/Roller/Profile/GoalVelocityRPS", 0.0);
      Logger.recordOutput("Intake/Roller/Profile/GoalVelocityRadPerSec", 0.0);
    }
  }

  /** Run roller at velocity (rotations per second) using torque current control */
  public void runVelocity(double velocityRPS) {
    if (!closedLoop) {
      // Seed profile state from measured velocity for a bumpless transfer to closed-loop.
      setpoint = new TrapezoidProfile.State(0.0, getMeasuredVelocityRPS());
      targetPositionRot = setpoint.position;
    }

    closedLoop = true;
    targetVelocityRPS = velocityRPS;
  }

  /** Run roller at intake velocity */
  public void runIntake() {
    runVelocity(intakeVelocity.get());
  }

  /** Run roller at intake velocity and override manual mode output. */
  public void runIntakeOverrideManual() {
    manualOverrideActive = true;
    runIntake();
  }

  /** Run roller at velocity (rotations per minute) using torque current control */
  public void runVelocityRPM(double velocityRPM) {
    runVelocity(velocityRPM / 60.0);
  }

  /** Run roller at velocity and override manual mode output. */
  public void runVelocityOverrideManual(double velocityRPS) {
    manualOverrideActive = true;
    runVelocity(velocityRPS);
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
    closedLoop = false;
    targetVelocityRPS = 0.0;
    io.runVolts(volts);
  }

  /** Run roller at torque current */
  public void runTorqueCurrent(double current) {
    closedLoop = false;
    targetVelocityRPS = 0.0;
    io.runTorqueCurrent(current);
  }

  /** Stop the roller */
  public void stop() {
    closedLoop = false;
    targetVelocityRPS = 0.0;
    io.stop();
  }

  /** Stop roller and hold that stop even when manual mode is enabled. */
  public void stopOverrideManual() {
    manualOverrideActive = true;
    stop();
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

  private double getMeasuredVelocityRPS() {
    return Units.radiansToRotations(inputs.talonVelocityRadsPerSec);
  }
}

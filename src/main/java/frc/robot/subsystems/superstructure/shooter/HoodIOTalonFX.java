// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class HoodIOTalonFX implements HoodIO {
  public static final double reduction = 108;

  // Hardware
  private final TalonFX talon;
  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public HoodIOTalonFX(int canId, String canBus) {
    talon = new TalonFX(canId, canBus);

    // Configure motor
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    config.Feedback.SensorToMechanismRatio = reduction;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -60;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    torqueCurrent = talon.getTorqueCurrent();
    supplyCurrent = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, position, velocity, appliedVolts, torqueCurrent, temp);
    ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp)
            .isOK();

    inputs.motorConnected = connectedDebouncer.calculate(connected);
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void runOpenLoop(double output) {
    talon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    talon.setControl(
        positionTorqueCurrentRequest
            .withPosition(Units.radiansToRotations(positionRad))
            .withFeedForward(feedforward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    talon.getConfigurator().apply(config);
  }

  @Override
  public void setPosition(double radians) {
    talon.setPosition(Units.radiansToRotations(radians));
  }
}

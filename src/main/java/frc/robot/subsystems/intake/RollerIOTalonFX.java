// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

/** Roller IO implementation using a Kraken X44. */
public class RollerIOTalonFX implements RollerIO {
  private final TalonFX talon;
  private final CANrange canRange;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final StatusSignal<Distance> distance;
  private final StatusSignal<Boolean> isDetected;
  private final StatusSignal<Time> measureTimestamp;

  // Control requests
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public RollerIOTalonFX(int canId, int canRangeId, String canBus) {
    talon = new TalonFX(canId, canBus);
    canRange = new CANrange(canRangeId, canBus);

    // Configure for Kraken X44 with velocity control
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    // Velocity PID defaults for Kraken X44
    config.Slot0 = new Slot0Configs().withKP(0.5).withKI(0).withKD(0).withKS(0.1).withKV(0.12);
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
    ProximityParamsConfigs proximityParamsConfigs = new ProximityParamsConfigs();
    proximityParamsConfigs.withProximityThreshold(0.2);
    proximityParamsConfigs.withMinSignalStrengthForValidMeasurement(10000);
    canRangeConfig.withProximityParams(proximityParamsConfigs);
    tryUntilOk(5, () -> canRange.getConfigurator().apply(canRangeConfig));

    distance = canRange.getDistance();
    isDetected = canRange.getIsDetected();
    measureTimestamp = canRange.getMeasurementTime();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                distance,
                isDetected,
                measureTimestamp));
    ParentDevice.optimizeBusUtilizationForAll(canRange, talon);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.talonConnected =
        connectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
                .isOK());
    inputs.CANRangeConnected =
        connectedDebouncer.calculate(BaseStatusSignal.refreshAll(distance, isDetected).isOK());
    inputs.measuredTimestamp = measureTimestamp.getValueAsDouble();
    inputs.hasCoral = isDetected.getValue();
    inputs.talonPositionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.talonVelocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.talonAppliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.talonSupplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.talonTorqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.talonTempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void runVelocity(double velocityRPS) {
    talon.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRPS));
  }

  @Override
  public void runTorqueCurrent(double current) {
    talon.setControl(torqueCurrentFOC.withOutput(current));
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () ->
                tryUntilOk(
                    5,
                    () ->
                        talon.setNeutralMode(
                            enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast)))
        .start();
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double kS, double kV) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }
}

// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class PivotIOTalonFX implements PivotIO {
  private static final Rotation2d offset = new Rotation2d();

  // Gear train: Motor → 22t gear → 30t gear → 12t sprocket → 22t sprocket → 23:1 cycloidal
  // Total reduction = (30/22) × (22/12) × 23 = 57.5:1
  public static final double gearRatio = 30.0 / 22.0;
  public static final double sprocketRatio = 22.0 / 12.0;
  public static final double cycloidalRatio = 23.0;
  public static final double reduction = gearRatio * sprocketRatio * cycloidalRatio; // 57.5:1

  // Hardware
  private final TalonFX talon;

  // Config
  private final TalonFXConfiguration Config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<Angle> internalPosition;
  private final StatusSignal<Angle> encoderAbsolutePosition;
  private final StatusSignal<Angle> encoderRelativePosition;
  private final StatusSignal<AngularVelocity> Velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temp;

  // Control Requests
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  // Connected debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

  public PivotIOTalonFX(int canId, String canBus) {
    talon = new TalonFX(canId, canBus);

    // Configure  motor
    Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    Config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    Config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    Config.Feedback.SensorToMechanismRatio = reduction;
    Config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    Config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    Config.CurrentLimits.StatorCurrentLimit = 40.0;
    Config.CurrentLimits.StatorCurrentLimitEnable = true;
    Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> talon.getConfigurator().apply(Config, 0.25));

    // Home to stowed position (90 degrees)
    talon.setPosition(Degree.of(90));

    // Get and set status signals
    internalPosition = talon.getPosition();
    encoderAbsolutePosition = talon.getPosition();
    encoderRelativePosition = talon.getPosition();
    Velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getStatorCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        internalPosition,
        encoderAbsolutePosition,
        encoderRelativePosition,
        Velocity,
        appliedVolts,
        current,
        temp);
    ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    // Refresh status signals and check if hardware connected
    boolean motorConnected =
        BaseStatusSignal.refreshAll(internalPosition, Velocity, appliedVolts, current, temp).isOK();
    boolean encoderConnected =
        BaseStatusSignal.refreshAll(internalPosition, Velocity, appliedVolts, current, temp).isOK();

    inputs.motorConnected = motorConnectedDebouncer.calculate(motorConnected);
    inputs.encoderConnected = encoderConnectedDebouncer.calculate(encoderConnected);
    // Pivot inputs
    inputs.internalPosition = Rotation2d.fromRadians(internalPosition.getValue().in(Radian));
    inputs.encoderAbsolutePosition =
        Rotation2d.fromRotations(encoderAbsolutePosition.getValueAsDouble()).minus(offset);
    inputs.encoderRelativePosition =
        encoderRelativePosition.getValue().in(Radians) - offset.getRadians();
    inputs.velocityRadPerSec = Velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVolts.getValue().in(Volts);
    inputs.currentAmps = current.getValue().in(Amps);
    inputs.tempCelsius = temp.getValue().in(Celsius);
  }

  @Override
  public void runOpenLoop(double output) {
    talon.setControl(torqueCurrentFOC.withOutput(output));
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
  public void runPosition(Rotation2d position, double feedforward) {
    talon.setControl(
        positionTorqueCurrentFOC
            .withPosition(position.getRotations())
            .withFeedForward(feedforward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    Config.Slot0.kP = kP;
    Config.Slot0.kI = kI;
    Config.Slot0.kD = kD;
    tryUntilOk(5, () -> talon.getConfigurator().apply(Config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              Config.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> talon.getConfigurator().apply(Config));
            })
        .start();
  }

  @Override
  public void setPosition(double degrees) {
    talon.setPosition(degrees);
  }
}

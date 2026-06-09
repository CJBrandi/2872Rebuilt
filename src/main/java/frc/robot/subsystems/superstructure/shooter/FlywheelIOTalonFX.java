package frc.robot.subsystems.superstructure.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonDevice;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.util.OrchestraInstrumentProvider;
import java.util.List;

/** Flywheel IO implementation using TalonFX (Falcon 500 / Kraken X60). */
public class FlywheelIOTalonFX implements FlywheelIO, OrchestraInstrumentProvider {
  // stepUp = flywheel speed / motor speed, but Talon expects sensor/mechanism ratio.
  // Integrated sensor is on the motor, so sensor/mechanism = 1 / stepUp.
  private static final double sensorToMechanismRatio =
      1.0 / Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants.stepUp;

  // Hardware
  private final TalonFX talon;
  private final TalonFX followerTalon;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

  // Control requests
  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public FlywheelIOTalonFX(int canId, int followerCanId) {
    this(canId, followerCanId, "");
  }

  public FlywheelIOTalonFX(int canId, int followerCanId, String canBus) {
    talon = new TalonFX(canId, canBus);
    followerTalon = new TalonFX(followerCanId, canBus);

    // Configure motor
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0 = new Slot0Configs().withKS(0).withKV(0).withKP(0).withKI(0).withKD(0);
    config.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> followerTalon.getConfigurator().apply(config, 0.25));
    followerTalon.setControl(new Follower(talon.getDeviceID(), MotorAlignmentValue.Aligned));

    // Status signals
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    torqueCurrent = talon.getTorqueCurrent();
    supplyCurrent = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();
    followerAppliedVolts = followerTalon.getMotorVoltage();
    followerTorqueCurrent = followerTalon.getTorqueCurrent();
    followerSupplyCurrent = followerTalon.getSupplyCurrent();
    followerTemp = followerTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0,
        velocity,
        appliedVolts,
        torqueCurrent,
        supplyCurrent,
        temp,
        followerAppliedVolts,
        followerTorqueCurrent,
        followerSupplyCurrent,
        followerTemp);
    ParentDevice.optimizeBusUtilizationForAll(talon, followerTalon);
  }

  @Override
  public void addOrchestraInstruments(List<CommonDevice> instruments) {
    instruments.add(talon);
    instruments.add(followerTalon);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    boolean motorConnected =
        BaseStatusSignal.refreshAll(velocity, appliedVolts, torqueCurrent, supplyCurrent, temp)
            .isOK();
    boolean followerConnected =
        BaseStatusSignal.refreshAll(
                followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent, followerTemp)
            .isOK();

    inputs.motorConnected = motorConnectedDebouncer.calculate(motorConnected && followerConnected);
    inputs.encoderConnected = encoderConnectedDebouncer.calculate(motorConnected);
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps =
        Math.abs(torqueCurrent.getValueAsDouble())
            + Math.abs(followerTorqueCurrent.getValueAsDouble());
    inputs.tempCelsius = Math.max(temp.getValueAsDouble(), followerTemp.getValueAsDouble());
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
  public void runVelocity(double radsPerSec) {
    talon.setControl(
        velocityTorqueCurrentRequest.withVelocity(Units.radiansToRotations(radsPerSec)));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setFF(double kS, double kV) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              NeutralModeValue mode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              talon.setNeutralMode(mode);
              followerTalon.setNeutralMode(mode);
            })
        .start();
  }
}

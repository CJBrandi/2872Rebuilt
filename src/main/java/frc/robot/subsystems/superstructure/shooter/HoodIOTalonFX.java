package frc.robot.subsystems.superstructure.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/** Hood IO implementation using TalonFX (Falcon 500). */
public class HoodIOTalonFX implements HoodIO {
  private static final double GEAR_RATIO = Constants.ShooterConstants.hoodReduction;

  private final TalonFX talon;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temperature;

  private final Debouncer connectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public HoodIOTalonFX(int canId) {
    this(canId, "");
  }

  public HoodIOTalonFX(int canId, String canBus) {
    talon = new TalonFX(canId, canBus);

    // Configure motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Gear ratio: motor rotations to mechanism rotations
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // PID gains (slot 0)
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;

    // Soft limits (in mechanism rotations)
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(Math.toRadians(70.0));
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(Math.toRadians(20.0));

    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Create status signals
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getStatorCurrent();
    temperature = talon.getDeviceTemp();

    // Set update frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, current, temperature);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current, temperature);

    inputs.motorConnected = connectedDebouncer.calculate(status.isOK());
    inputs.encoderConnected = inputs.motorConnected;
    inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.tempCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void runOpenLoop(double output) {
    talon.set(output);
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
        positionRequest.withPosition(position.getRotations()).withFeedForward(feedforward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    talon.getConfigurator().apply(config);
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

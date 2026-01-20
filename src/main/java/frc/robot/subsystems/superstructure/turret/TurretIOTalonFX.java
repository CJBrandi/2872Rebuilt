package frc.robot.subsystems.superstructure.turret;

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

/** Turret IO implementation using TalonFX (Kraken X60). */
public class TurretIOTalonFX implements TurretIO {
  private static final double GEAR_RATIO = Constants.TurretConstants.reduction;

  // Turret limits in mechanism rotations
  private static final double MIN_ANGLE_ROTATIONS = 0.0;
  private static final double MAX_ANGLE_ROTATIONS = Units.radiansToRotations((3 * Math.PI) / 2);

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

  public TurretIOTalonFX(int canId) {
    this(canId, "");
  }

  public TurretIOTalonFX(int canId, String canBus) {
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

    // Soft limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE_ROTATIONS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE_ROTATIONS;

    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Create status signals
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getStatorCurrent();
    temperature = talon.getDeviceTemp();

    // Set update frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, position, velocity, appliedVolts, current, temperature);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current, temperature);

    inputs.motorConnected = connectedDebouncer.calculate(status.isOK());
    inputs.encoderConnected = inputs.motorConnected;
    inputs.motorEncoderPosition = Rotation2d.fromRotations(position.getValueAsDouble());
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
  public void setPosition(double degrees) {
    talon.setPosition(Units.degreesToRotations(degrees));
  }
}

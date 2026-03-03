package frc.robot.subsystems.superstructure.turret;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/** Turret IO implementation using TalonFX (Kraken X60). */
public class TurretIOTalonFX implements TurretIO {
  private static final double GEAR_RATIO =
      Constants.SuperstructureConstants.TurretConstants.reduction;

  private final TalonFX talon;
  DigitalInput leftHall = new DigitalInput(2);
  DigitalInput middleHall = new DigitalInput(0);
  DigitalInput rightHall = new DigitalInput(1);

  // Hall sensors are wired active-low: false means magnet present.
  private static boolean isHallTriggered(DigitalInput input) {
    return !input.get();
  }

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temperature;

  private final Debouncer connectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public TurretIOTalonFX(int canId, String canBus) {
    talon = new TalonFX(canId, canBus);

    // Configure motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Gear ratio: motor rotations to mechanism rotations
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;

    // PID gains (slot 0)
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;

    // Disable soft limits - turret uses continuous rotation with wrap-around logic
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Create status signals
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getStatorCurrent();
    temperature = talon.getDeviceTemp();

    // Set update frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, position, velocity, appliedVolts, current, temperature);
    talon.optimizeBusUtilization();

    talon.setPosition(Units.degreesToRotations(-90));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current, temperature);

    inputs.hallEffectState[0] = isHallTriggered(leftHall);
    inputs.hallEffectState[1] = isHallTriggered(middleHall);
    inputs.hallEffectState[2] = isHallTriggered(rightHall);

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
  public void runCurrent(double currentAmps) {
    talon.setControl(currentRequest.withOutput(currentAmps));
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
    var slot0Config = new com.ctre.phoenix6.configs.Slot0Configs();
    talon.getConfigurator().refresh(slot0Config);
    slot0Config.kP = kP;
    slot0Config.kI = kI;
    slot0Config.kD = kD;
    talon.getConfigurator().apply(slot0Config);
  }

  @Override
  public void setPosition(double degrees) {
    talon.setPosition(Units.degreesToRotations(degrees));
  }
}

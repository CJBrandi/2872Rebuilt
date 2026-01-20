package frc.robot.subsystems.superstructure.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;

/** Flywheel IO implementation using REV SparkFlex with NEO Vortex motor. */
public class FlywheelIOSparkFlex implements FlywheelIO {
  private static final double GEAR_RATIO = 1.0 / Constants.ShooterConstants.flywheelStepUp;

  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  private final Debouncer connectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public FlywheelIOSparkFlex(int canId) {
    motor = new SparkFlex(canId, SparkLowLevel.MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();

    // Configure motor
    SparkFlexConfig config = new SparkFlexConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(60)
        .voltageCompensation(12.0);

    config
        .encoder
        .positionConversionFactor(GEAR_RATIO * 2 * Math.PI) // rotations -> radians (output)
        .velocityConversionFactor(GEAR_RATIO * 2 * Math.PI / 60.0); // RPM -> rad/s (output)

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0, 0.0, 0.0);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.motorConnected = connectedDebouncer.calculate(!motor.hasActiveFault());
    inputs.encoderConnected = true;
    inputs.velocityRadPerSec = encoder.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void runOpenLoop(double output) {
    motor.set(output);
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void runVelocity(double radsPerSec, double feedforward) {
    controller.setSetpoint(radsPerSec, SparkBase.ControlType.kVelocity);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.pid(kP, kI, kD);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(enabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}

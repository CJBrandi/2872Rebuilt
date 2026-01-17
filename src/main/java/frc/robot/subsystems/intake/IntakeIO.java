package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public boolean motorConnected = true;
    public boolean encoderConnected = true;

    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runVelocity(double radsPerSec, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}

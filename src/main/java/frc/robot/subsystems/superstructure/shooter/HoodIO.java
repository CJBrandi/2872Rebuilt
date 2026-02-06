package frc.robot.subsystems.superstructure.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  class HoodIOInputs {
    public boolean motorConnected = true;
    public boolean encoderConnected = true;

    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void stop() {}

  default void runVolts(double volts) {}

  default void runPosition(double positionRad, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}

  default void setPosition(double radians) {}
}

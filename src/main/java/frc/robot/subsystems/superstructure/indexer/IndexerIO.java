package frc.robot.subsystems.superstructure.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public boolean motorConnected = true;
    public boolean followerConnected = true;
    public boolean encoderConnected = true;

    public double velocityRadPerSec = 0.0;
    public double auxVelocityRPM = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] currentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runVelocity(double radsPerSec) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setFF(double kS, double kV) {}

  default void setAuxIndexerVelocityRPM(double velocityRPM) {}

  default void setAuxIndexerPIDF(double kP, double kI, double kD, double kF) {}

  default void setBrakeMode(boolean enabled) {}
}

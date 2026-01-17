package frc.robot.subsystems.superstructure.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/kP", 0.05);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Indexer/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/kD", 0.0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Indexer/kV", 0.02);

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  @Getter private double velocitySetpointRadPerSec = 0.0;
  private boolean closedLoop = false;

  @Getter
  @AutoLogOutput(key = "Indexer/AtSetpoint")
  private boolean atSetpoint = false;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

    if (closedLoop) {
      double toleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(50);
      atSetpoint =
          EqualsUtil.epsilonEquals(
              inputs.velocityRadPerSec, velocitySetpointRadPerSec, toleranceRadPerSec);
    } else {
      atSetpoint = false;
    }

    Logger.recordOutput("Indexer/Profile/SetpointVelocityRadPerSec", velocitySetpointRadPerSec);
    Logger.recordOutput(
        "Indexer/Profile/SetpointVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(velocitySetpointRadPerSec));
    Logger.recordOutput("Indexer/Profile/ActualVelocityRadPerSec", inputs.velocityRadPerSec);
    Logger.recordOutput(
        "Indexer/Profile/ActualVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec));
    Logger.recordOutput(
        "Indexer/Profile/VelocityErrorRadPerSec",
        velocitySetpointRadPerSec - inputs.velocityRadPerSec);
    Logger.recordOutput("Indexer/ClosedLoop", closedLoop);
  }

  public void runVelocity(double velocityRadPerSec) {
    closedLoop = true;
    velocitySetpointRadPerSec = velocityRadPerSec;
    double feedforward = kV.get() * velocityRadPerSec;
    io.runVelocity(velocityRadPerSec, feedforward);
  }

  public void runVelocityRPM(double velocityRPM) {
    runVelocity(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM));
  }

  public void runVolts(double volts) {
    closedLoop = false;
    velocitySetpointRadPerSec = 0.0;
    io.runVolts(volts);
  }

  public void runOpenLoop(double output) {
    closedLoop = false;
    velocitySetpointRadPerSec = 0.0;
    io.runOpenLoop(output);
  }

  public void stop() {
    closedLoop = false;
    velocitySetpointRadPerSec = 0.0;
    io.stop();
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public boolean isMotorConnected() {
    return motorConnectedDebouncer.calculate(inputs.motorConnected);
  }
}

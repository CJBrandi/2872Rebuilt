package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.05);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.02);

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  @Getter private double velocitySetpointRadPerSec = 0.0;
  private boolean closedLoop = false;

  @Getter
  @AutoLogOutput(key = "Shooter/AtSetpoint")
  private boolean atSetpoint = false;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Update PID gains if changed
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

    // Check if at setpoint (within tolerance)
    if (closedLoop) {
      double toleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(50);
      atSetpoint =
          EqualsUtil.epsilonEquals(
              inputs.velocityRadPerSec, velocitySetpointRadPerSec, toleranceRadPerSec);
    } else {
      atSetpoint = false;
    }

    Logger.recordOutput("Shooter/Profile/SetpointVelocityRadPerSec", velocitySetpointRadPerSec);
    Logger.recordOutput(
        "Shooter/Profile/SetpointVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(velocitySetpointRadPerSec));
    Logger.recordOutput("Shooter/Profile/ActualVelocityRadPerSec", inputs.velocityRadPerSec);
    Logger.recordOutput(
        "Shooter/Profile/ActualVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec));
    Logger.recordOutput(
        "Shooter/Profile/VelocityErrorRadPerSec",
        velocitySetpointRadPerSec - inputs.velocityRadPerSec);
    Logger.recordOutput("Shooter/ClosedLoop", closedLoop);
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

package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 50);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0.0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/kV", 0.02);
  public static final LoggedTunableNumber targetVelocityRadPerSec =
      new LoggedTunableNumber("Intake/targetVelocityRadPerSec", 50);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private boolean closedLoop = false;

  @Getter
  @AutoLogOutput(key = "Intake/AtSetpoint")
  private boolean atSetpoint = false;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

    if (closedLoop) {
      double toleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(50);
      atSetpoint =
          EqualsUtil.epsilonEquals(
              inputs.velocityRadPerSec, targetVelocityRadPerSec.get(), toleranceRadPerSec);
    } else {
      atSetpoint = false;
    }

    Logger.recordOutput("Intake/Profile/SetpointVelocityRadPerSec", targetVelocityRadPerSec.get());
    Logger.recordOutput(
        "Intake/Profile/SetpointVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(targetVelocityRadPerSec.get()));
    Logger.recordOutput("Intake/Profile/ActualVelocityRadPerSec", inputs.velocityRadPerSec);
    Logger.recordOutput(
        "Intake/Profile/ActualVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec));
    Logger.recordOutput(
        "Intake/Profile/VelocityErrorRadPerSec",
        targetVelocityRadPerSec.get() - inputs.velocityRadPerSec);
    Logger.recordOutput("Intake/ClosedLoop", closedLoop);
  }

  public void runIntake() {
    closedLoop = true;
    double feedforward = kV.get() * targetVelocityRadPerSec.get();
    io.runVelocity(targetVelocityRadPerSec.get(), feedforward);
  }

  public void runVelocity(double velocityRadPerSec) {
    closedLoop = true;
    double feedforward = kV.get() * velocityRadPerSec;
    io.runVelocity(velocityRadPerSec, feedforward);
  }

  public void runVelocityRPM(double velocityRPM) {
    runVelocity(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM));
  }

  public void runVolts(double volts) {
    closedLoop = false;
    io.runVolts(volts);
  }

  public void runOpenLoop(double output) {
    closedLoop = false;
    io.runOpenLoop(output);
  }

  public void stop() {
    closedLoop = false;
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

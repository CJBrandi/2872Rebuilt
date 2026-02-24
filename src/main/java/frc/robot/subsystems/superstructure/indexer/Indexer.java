package frc.robot.subsystems.superstructure.indexer;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Indexer {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Superstructure/Indexer/kP", 3.0);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Superstructure/Indexer/kI", 0.0);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Superstructure/Indexer/kD", 0.0);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Superstructure/Indexer/kS", 0.0);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Superstructure/Indexer/kV", 0.0);

  private static final LoggedTunableNumber maxVelocityRadPerSec =
      new LoggedTunableNumber("Superstructure/Indexer/MaxVelocityRadPerSec", 200.0);
  private static final LoggedTunableNumber maxAccelerationRadPerSec2 =
      new LoggedTunableNumber("Superstructure/Indexer/MaxAccelerationRadPerSec2", 400.0);

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private TrapezoidProfile profile;
  @Getter private TrapezoidProfile.State setpoint;
  private double targetPositionRad = 0.0;

  @Getter private double targetVelocityRadPerSec = 0.0;
  private boolean closedLoop = false;

  @Getter private boolean atSetpoint = false;

  public Indexer(IndexerIO io) {
    this.io = io;
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityRadPerSec.get(), maxAccelerationRadPerSec2.get()));
    setpoint = new TrapezoidProfile.State(0.0, 0.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Indexer", inputs);

    // Update gains/config when tunables change
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(hashCode(), () -> io.setFF(kS.get(), kV.get()), kS, kV);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            profile =
                new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                        maxVelocityRadPerSec.get(), maxAccelerationRadPerSec2.get())),
        maxVelocityRadPerSec,
        maxAccelerationRadPerSec2);

    if (closedLoop) {
      // Move a virtual position target at the desired steady-state velocity and profile towards it.
      targetPositionRad += targetVelocityRadPerSec * Constants.loopPeriodSecs;
      var goalState = new TrapezoidProfile.State(targetPositionRad, targetVelocityRadPerSec);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      io.runVelocity(setpoint.velocity);

      atSetpoint =
          EqualsUtil.epsilonEquals(
              inputs.velocityRadPerSec,
              setpoint.velocity,
              Units.rotationsPerMinuteToRadiansPerSecond(50.0));
    } else {
      atSetpoint = false;
    }

    Logger.recordOutput(
        "Superstructure/Indexer/TargetVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(targetVelocityRadPerSec));
    Logger.recordOutput(
        "Superstructure/Indexer/SetpointVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(setpoint.velocity));
    Logger.recordOutput(
        "Superstructure/Indexer/MeasuredVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec));
    Logger.recordOutput("Superstructure/Indexer/AtSetpoint", atSetpoint);
    Logger.recordOutput("Superstructure/Indexer/ClosedLoop", closedLoop);
  }

  /** Runs closed-loop velocity (rad/s). */
  public void runVelocity(double velocityRadPerSec) {
    if (!closedLoop) {
      // Seed profile state from measured velocity for bumpless transfer.
      setpoint = new TrapezoidProfile.State(0.0, inputs.velocityRadPerSec);
      targetPositionRad = setpoint.position;
    }

    closedLoop = true;
    targetVelocityRadPerSec = velocityRadPerSec;
  }

  /** Runs closed-loop velocity (RPM). */
  public void runVelocityRPM(double velocityRPM) {
    runVelocity(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM));
  }

  public void runVolts(double volts) {
    closedLoop = false;
    targetVelocityRadPerSec = 0.0;
    io.runVolts(volts);
  }

  public void runOpenLoop(double output) {
    closedLoop = false;
    targetVelocityRadPerSec = 0.0;
    io.runOpenLoop(output);
  }

  /** Decelerates to zero using the trapezoidal velocity profile. */
  public void stop() {
    runVelocity(0.0);
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public boolean isMotorConnected() {
    return inputs.motorConnected;
  }
}

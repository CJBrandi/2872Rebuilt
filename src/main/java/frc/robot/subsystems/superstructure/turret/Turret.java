package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.EqualsUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", 5.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 0.1);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", 0.2);
  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Turret/MaxVelocityDegreesPerSec", 1500.0);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Turret/MaxAccelerationDegreesPerSec2", 2500.0);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Turret/StaticCharacterizationVelocityThresh", 0.1);
  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private TrapezoidProfile profile;
  @Getter private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  @Getter
  @AutoLogOutput(key = "Turret/Profile/AtGoal")
  private boolean atGoal = false;

  public Turret(TurretIO io) {
    turretIO = io;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
  }

  @Override
  public void periodic() {
    turretIO.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    double targetAngleRad =
        Math.atan2(
            FieldConstants.HUB.getY() - RobotState.getInstance().getPose().getY(),
            FieldConstants.HUB.getX() - RobotState.getInstance().getPose().getX());
    double robotAngleRad = RobotState.getInstance().getPose().getRotation().getRadians();
    double turretGoalAngleRad =
        MathUtil.inputModulus(targetAngleRad - robotAngleRad, 0, 2 * Math.PI);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      turretIO.setPID(kP.get(), 0.0, kD.get());
    }
    if (maxVelocityDegPerSec.hasChanged(hashCode())
        || maxAccelerationDegPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(maxVelocityDegPerSec.get()),
                  Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    }

    var goalState =
        new TrapezoidProfile.State(MathUtil.clamp(turretGoalAngleRad, 0, (3 * Math.PI) / 2), 0.0);
    setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);

    turretIO.runPosition(
        Rotation2d.fromRadians(setpoint.position), kS.get() * Math.signum(setpoint.velocity));
    atGoal =
        EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
            && EqualsUtil.epsilonEquals(setpoint.velocity, 0.0);

    Logger.recordOutput("Turret/Profile/SetpointAngleRad", setpoint.position);
    Logger.recordOutput("Turret/Profile/SetpointAngleRadPerSec", setpoint.velocity);
    Logger.recordOutput("Turret/Profile/GoalAngleRad", goalState.position);

    TurretVisualizer.update(setpoint.position);
  }

  /** Resets the turret encoder position and profile setpoint to the specified angle in degrees. */
  public void resetPosition(double degrees) {
    turretIO.setPosition(degrees);
    setpoint = new TrapezoidProfile.State(Math.toRadians(degrees), 0.0);
  }
}

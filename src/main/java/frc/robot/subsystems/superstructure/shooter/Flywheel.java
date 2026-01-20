package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Flywheel control for the shooter. Handles velocity control with slew rate limiting. This is not a
 * Subsystem - it's managed by Shooter.
 */
public class Flywheel {

  // Wheel radius in meters (4 inch wheels)
  private static final double WHEEL_RADIUS = Units.inchesToMeters(4.0);

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/Flywheel/kP", 0.05);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/Flywheel/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Flywheel/kD", 0.0);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/Flywheel/kV", 0.02);

  // Efficiency coefficient: accounts for friction between wheels and ball
  // exitVelocity = wheelSurfaceVelocity * efficiency
  private static final LoggedTunableNumber efficiency =
      new LoggedTunableNumber("Shooter/Flywheel/Efficiency", 0.85);

  // Slew rate limiter parameters (rad/s per second)
  private static final LoggedTunableNumber maxAccelRadPerSec2 =
      new LoggedTunableNumber("Shooter/Flywheel/MaxAccelRadPerSec2", 300.0);
  private static final LoggedTunableNumber maxDecelRadPerSec2 =
      new LoggedTunableNumber("Shooter/Flywheel/MaxDecelRadPerSec2", 500.0);

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private SlewRateLimiter velocityLimiter;

  @Getter private double targetExitVelocityMps = 0.0;
  private double targetWheelVelocityRadPerSec = 0.0;
  private double velocitySetpointRadPerSec = 0.0;
  private boolean closedLoop = false;

  @Getter private boolean atSetpoint = false;

  public Flywheel(FlywheelIO io) {
    this.io = io;
    velocityLimiter = new SlewRateLimiter(maxAccelRadPerSec2.get(), -maxDecelRadPerSec2.get(), 0.0);
  }

  /** Called by Shooter.periodic() */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);

    // Update PID gains if changed
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

    // Update slew rate limiter if parameters changed
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            velocityLimiter =
                new SlewRateLimiter(
                    maxAccelRadPerSec2.get(), -maxDecelRadPerSec2.get(), velocitySetpointRadPerSec),
        maxAccelRadPerSec2,
        maxDecelRadPerSec2);

    // Run velocity control if in closed loop mode
    if (closedLoop) {
      // Convert exit velocity to wheel velocity accounting for efficiency
      if (targetExitVelocityMps > 0.0) {
        double wheelSurfaceVelocityMps = targetExitVelocityMps / efficiency.get();
        targetWheelVelocityRadPerSec = wheelSurfaceVelocityMps / WHEEL_RADIUS;
      } else {
        targetWheelVelocityRadPerSec = 0.0;
      }

      // Apply slew rate limiting
      velocitySetpointRadPerSec = velocityLimiter.calculate(targetWheelVelocityRadPerSec);

      double feedforward = kV.get() * velocitySetpointRadPerSec;
      io.runVelocity(velocitySetpointRadPerSec, feedforward);
    }

    // Check if at setpoint
    if (closedLoop) {
      double toleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(50);
      atSetpoint =
          EqualsUtil.epsilonEquals(
              inputs.velocityRadPerSec, targetWheelVelocityRadPerSec, toleranceRadPerSec);
    } else {
      atSetpoint = false;
    }

    // Log values
    Logger.recordOutput("Shooter/Flywheel/TargetExitVelocityMps", targetExitVelocityMps);
    Logger.recordOutput("Shooter/Flywheel/Efficiency", efficiency.get());
    Logger.recordOutput(
        "Shooter/Flywheel/TargetWheelVelocityRadPerSec", targetWheelVelocityRadPerSec);
    Logger.recordOutput("Shooter/Flywheel/SetpointVelocityRadPerSec", velocitySetpointRadPerSec);
    Logger.recordOutput(
        "Shooter/Flywheel/SetpointVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(velocitySetpointRadPerSec));
    Logger.recordOutput("Shooter/Flywheel/ActualVelocityRadPerSec", inputs.velocityRadPerSec);
    Logger.recordOutput(
        "Shooter/Flywheel/ActualVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec));
    Logger.recordOutput("Shooter/Flywheel/AtSetpoint", atSetpoint);
    Logger.recordOutput("Shooter/Flywheel/ClosedLoop", closedLoop);
  }

  /** Sets the target exit velocity. */
  public void setTargetExitVelocity(double velocityMps) {
    closedLoop = true;
    this.targetExitVelocityMps = velocityMps;
  }

  /** Directly runs at a wheel velocity (bypasses exit velocity conversion). */
  public void runWheelVelocity(double velocityRadPerSec) {
    closedLoop = true;
    targetExitVelocityMps = 0.0;
    targetWheelVelocityRadPerSec = velocityRadPerSec;
  }

  public void runVolts(double volts) {
    closedLoop = false;
    targetExitVelocityMps = 0.0;
    targetWheelVelocityRadPerSec = 0.0;
    velocityLimiter.reset(inputs.velocityRadPerSec);
    io.runVolts(volts);
  }

  public void runOpenLoop(double output) {
    closedLoop = false;
    targetExitVelocityMps = 0.0;
    targetWheelVelocityRadPerSec = 0.0;
    velocityLimiter.reset(inputs.velocityRadPerSec);
    io.runOpenLoop(output);
  }

  public void stop() {
    closedLoop = true;
    targetExitVelocityMps = 0.0;
    targetWheelVelocityRadPerSec = 0.0;
  }

  /** Returns the current wheel angular velocity in rad/s. */
  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  /** Returns the estimated ball exit velocity in m/s. */
  public double getExitVelocityMps() {
    return inputs.velocityRadPerSec * WHEEL_RADIUS * efficiency.get();
  }

  public boolean isMotorConnected() {
    return inputs.motorConnected;
  }
}

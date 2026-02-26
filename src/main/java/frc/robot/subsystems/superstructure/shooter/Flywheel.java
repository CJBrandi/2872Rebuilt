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

  // Wheel radius in meters (4 inch diameter wheels)
  private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Flywheel/kP", 8);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/Flywheel/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Flywheel/kD", 0.0);
  // Feedforward gains for torque current control (Amps)
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Flywheel/kS", 0.0);
  // kV in Amps per rotation per second
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/Flywheel/kV", 0.45);

  // Efficiency coefficient: accounts for friction between wheels and ball
  // exitVelocity = wheelSurfaceVelocity * efficiency
  private static final LoggedTunableNumber efficiency =
      new LoggedTunableNumber("Shooter/Flywheel/Efficiency", 0.62);

  // Slew rate limiter parameters (rad/s per second)
  private static final LoggedTunableNumber maxAccelRadPerSec2 =
      new LoggedTunableNumber("Shooter/Flywheel/MaxAccelRadPerSec2", 500.0);
  private static final LoggedTunableNumber maxDecelRadPerSec2 =
      new LoggedTunableNumber("Shooter/Flywheel/MaxDecelRadPerSec2", 800.0);

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private SlewRateLimiter velocityLimiter;

  @Getter private double targetExitVelocityMps = 0.0;
  private double targetWheelVelocityRadPerSec = 0.0;
  private double velocitySetpointRadPerSec = 0.0;
  private boolean closedLoop = false;
  private boolean directWheelVelocity = false;

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

    // Update feedforward gains if changed
    LoggedTunableNumber.ifChanged(hashCode(), () -> io.setFF(kS.get(), kV.get()), kS, kV);

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
      // Only convert from exit velocity if not using direct wheel velocity
      if (!directWheelVelocity) {
        double wheelSurfaceVelocityMps = targetExitVelocityMps / efficiency.get();
        targetWheelVelocityRadPerSec = wheelSurfaceVelocityMps / WHEEL_RADIUS;
      }

      // Apply slew rate limiting
      velocitySetpointRadPerSec = velocityLimiter.calculate(targetWheelVelocityRadPerSec);

      io.runVelocity(velocitySetpointRadPerSec);
    }

    // Check if at setpoint (compare against slew-limited setpoint, not final target)
    if (closedLoop) {
      double toleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(50);
      atSetpoint =
          EqualsUtil.epsilonEquals(
              inputs.velocityRadPerSec, velocitySetpointRadPerSec, toleranceRadPerSec);
    } else {
      atSetpoint = false;
    }

    // Log values
    Logger.recordOutput("Shooter/Flywheel/TargetExitVelocityMps", targetExitVelocityMps);
    Logger.recordOutput("Shooter/Flywheel/Efficiency", efficiency.get());
    Logger.recordOutput(
        "Shooter/Flywheel/TargetWheelVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(targetWheelVelocityRadPerSec));
    Logger.recordOutput(
        "Shooter/Flywheel/SetpointVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(velocitySetpointRadPerSec));
    Logger.recordOutput(
        "Shooter/Flywheel/MeasuredVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec));
    Logger.recordOutput("Shooter/Flywheel/AtSetpoint", atSetpoint);
    Logger.recordOutput("Shooter/Flywheel/ClosedLoop", closedLoop);
  }

  /** Sets the target exit velocity. */
  public void setTargetExitVelocity(double velocityMps) {
    closedLoop = true;
    directWheelVelocity = false;
    this.targetExitVelocityMps = velocityMps;
  }

  /** Directly runs at a wheel velocity (bypasses exit velocity conversion). */
  public void runWheelVelocity(double velocityRadPerSec) {
    closedLoop = true;

    targetExitVelocityMps = 0.0;

    directWheelVelocity = true;

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

package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter coordinator. Receives goal values from Superstructure and delegates control to Flywheel
 * and Hood components. This is not a Subsystem - it's managed by Superstructure.
 */
@Getter
public class Shooter {

  // Manual mode tunable numbers (shared with Turret via same NT key)
  private static final LoggedTunableNumber manualModeEnabled =
      new LoggedTunableNumber("Manual/Enabled", 0.0);
  private static final LoggedTunableNumber manualFlywheelRPM =
      new LoggedTunableNumber("Manual/FlywheelRPM", 3000.0);
  private static final LoggedTunableNumber manualHoodAngleDeg =
      new LoggedTunableNumber("Manual/PitchDeg", 45.0);

  /** -- GETTER -- Returns the flywheel component for direct access if needed. */
  private final Flywheel flywheel;
  /** -- GETTER -- Returns the hood component for direct access if needed. */
  private final Hood hood;

  // Goal values set by Superstructure
  private double goalExitVelocityMps = 0.0;
  private double goalHoodAngleRad = Hood.getMinAngleRad();
  private double goalHoodVelocityRadPerSec = 0.0;

  public Shooter(FlywheelIO flywheelIO, HoodIO hoodIO) {
    this.flywheel = new Flywheel(flywheelIO);
    this.hood = new Hood(hoodIO);
  }

  public void periodic() {
    // Apply goals to components BEFORE running periodic control loops
    // This ensures the correct mode/targets are set before control runs
    if (manualModeEnabled.get() > 0.5) {
      // Manual mode: use tunable RPM and angle values with closed-loop control
      double manualVelocityRadPerSec =
          Units.rotationsPerMinuteToRadiansPerSecond(manualFlywheelRPM.get());
      flywheel.runWheelVelocity(manualVelocityRadPerSec);
      hood.setTargetAngle(Math.toRadians(manualHoodAngleDeg.get()), 0.0);
    } else {
      // Normal mode: use goals from Superstructure
      flywheel.setTargetExitVelocity(goalExitVelocityMps);
      hood.setTargetAngle(Math.toRadians(90) - goalHoodAngleRad, goalHoodVelocityRadPerSec);
    }

    flywheel.periodic();
    hood.periodic();

    Logger.recordOutput("Shooter/ManualMode", manualModeEnabled.get() > 0.5);
    Logger.recordOutput("Shooter/Manual/FlywheelRPM", manualFlywheelRPM.get());
    Logger.recordOutput("Shooter/Manual/HoodAngleDeg", manualHoodAngleDeg.get());
    Logger.recordOutput("Shooter/GoalExitVelocityMps", goalExitVelocityMps);
    Logger.recordOutput("Shooter/GoalHoodAngleRad", goalHoodAngleRad);
    Logger.recordOutput("Shooter/GoalHoodAngleDeg", Math.toDegrees(goalHoodAngleRad));
    Logger.recordOutput("Shooter/GoalHoodVelocityRadPerSec", goalHoodVelocityRadPerSec);
    Logger.recordOutput("Shooter/Ready", isReady());
  }

  /**
   * Sets the shooter goals. Called by Superstructure.
   *
   * @param exitVelocityMps Target ball exit velocity in m/s
   * @param hoodAngleRad Target hood angle in radians
   * @param hoodVelocityRadPerSec Hood feedforward velocity in rad/s
   */
  public void setGoals(double exitVelocityMps, double hoodAngleRad, double hoodVelocityRadPerSec) {
    this.goalExitVelocityMps = exitVelocityMps;
    this.goalHoodAngleRad = hoodAngleRad;
    this.goalHoodVelocityRadPerSec = hoodVelocityRadPerSec;
  }

  /**
   * Sets the shooter goals with no hood feedforward velocity.
   *
   * @param exitVelocityMps Target ball exit velocity in m/s
   * @param hoodAngleRad Target hood angle in radians
   */
  public void setGoals(double exitVelocityMps, double hoodAngleRad) {
    setGoals(exitVelocityMps, hoodAngleRad, 0.0);
  }

  /** Returns whether both flywheel and hood are at their setpoints. */
  @AutoLogOutput(key = "Shooter/Ready")
  public boolean isReady() {
    return flywheel.isAtSetpoint() && hood.isAtGoal();
  }

  /** Returns whether the flywheel is at its velocity setpoint. */
  public boolean isFlywheelReady() {
    return flywheel.isAtSetpoint();
  }

  /** Returns whether the hood is at its angle setpoint. */
  public boolean isHoodReady() {
    return hood.isAtGoal();
  }

  /** Returns the current ball exit velocity in m/s. */
  public double getExitVelocityMps() {
    return flywheel.getExitVelocityMps();
  }

  /** Returns the current hood angle in radians. */
  public double getHoodAngleRad() {
    return hood.getAngle().getRadians();
  }

  // ==================== Direct control methods (for testing/characterization) ====================

  /** Runs flywheel at specified voltage (bypasses goal system). */
  public void runFlywheelVolts(double volts) {
    goalExitVelocityMps = 0.0;
    flywheel.runVolts(volts);
  }

  /** Stops all motors (sets goals to zero/min positions). */
  public void stop() {
    goalExitVelocityMps = 0.0;
    goalHoodAngleRad = Hood.getMinAngleRad();
    goalHoodVelocityRadPerSec = 0.0;
  }

  // ==================== Hood Commands ====================

  /**
   * Creates a command for hood homing sequence.
   *
   * @return Command that homes the hood by running into the hard stop
   */
  public Command hoodHomingCommand() {
    return hood.homingSequence();
  }

  /**
   * Creates a command for hood static characterization.
   *
   * @param currentRampRateAmpsPerSec Rate at which to increase current (amps per second)
   * @return Command that runs the characterization
   */
  public Command hoodStaticCharacterizationCommand(double currentRampRateAmpsPerSec) {
    return hood.staticCharacterization(currentRampRateAmpsPerSec);
  }

  /** Returns whether the hood has been homed. */
  public boolean isHoodHomed() {
    return Constants.currentMode == Constants.Mode.SIM || hood.isHomed();
  }
}

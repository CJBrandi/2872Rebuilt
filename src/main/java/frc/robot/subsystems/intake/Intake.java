// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends SubsystemBase {
  // Angle constants: 0° = ground/deployed, 90° = stowed
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(90);
  public static final Rotation2d stowedAngle = Rotation2d.fromDegrees(90);
  public static final Rotation2d groundAngle = Rotation2d.fromDegrees(0);

  // Subsystems
  @Getter private final Pivot pivot;
  @Getter private final Roller roller;

  public Intake(PivotIO pivotIO, RollerIO rollerIO) {
    this.pivot = new Pivot(pivotIO);
    this.roller = new Roller(rollerIO);
  }

  public void periodic() {
    pivot.periodic();
    roller.periodic();
  }

  // ==================== Pivot Control ====================

  /** Set the pivot goal angle */
  public void setPivotGoal(Supplier<Rotation2d> goal) {
    pivot.setGoal(goal);
  }

  /** Set the pivot goal angle in radians */
  public void setPivotGoal(DoubleSupplier goalRad) {
    pivot.setGoal(goalRad);
  }

  /** Set pivot to stowed position (90 degrees) */
  public void stow() {
    pivot.setGoal(() -> stowedAngle);
  }

  /** Set pivot to ground/deployed position (0 degrees) */
  public void deploy() {
    pivot.setGoal(() -> groundAngle);
  }

  /** Check if pivot is at goal */
  @AutoLogOutput(key = "Intake/AtGoal")
  public boolean isAtGoal() {
    return pivot.isAtGoal();
  }

  /** Get current pivot angle */
  public Rotation2d getPivotAngle() {
    return pivot.getAngle();
  }

  /** Home the pivot to stowed position */
  public void homeToStowed() {
    pivot.homeToStowed();
  }

  /** Returns the homing sequence command for the pivot */
  public Command homingSequence() {
    return pivot.homingSequence();
  }

  /** Returns whether the pivot is homed */
  public boolean isHomed() {
    return pivot.isHomed();
  }

  /** Set overrides for pivot */
  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    pivot.setOverrides(coastOverride, disabledOverride);
  }

  /** Set E-stop state */
  public void setEStopped(boolean estopped) {
    pivot.setEStopped(estopped);
  }

  /** Check if should E-stop */
  public boolean shouldEStop() {
    return pivot.isShouldEStop();
  }

  // ==================== Roller Control ====================

  /** Run roller at intake velocity */
  public void runIntake() {
    roller.runIntake();
  }

  /** Run roller at eject velocity */
  public void runEject() {
    roller.runEject();
  }

  /** Run roller at hold velocity */
  public void runHold() {
    roller.runHold();
  }

  /** Run roller at specific velocity (RPS) */
  public void runRollerVelocity(double velocityRPS) {
    roller.runVelocity(velocityRPS);
  }

  /** Stop the roller */
  public void stopRoller() {
    roller.stop();
  }

  // ==================== Commands ====================

  /** Command to deploy intake and run rollers */
  public Command intakeCommand() {
    return Commands.runEnd(
        () -> {
          deploy();
          runIntake();
        },
        () -> {
          stow();
          stopRoller();
        });
  }

  /** Command to eject game piece */
  public Command ejectCommand() {
    return Commands.runEnd(this::runEject, this::stopRoller);
  }

  /** Command to stow the intake */
  public Command stowCommand() {
    return Commands.runOnce(this::stow);
  }

  /** Command to deploy the intake */
  public Command deployCommand() {
    return Commands.runOnce(this::deploy);
  }

  /** Static characterization command for pivot */
  public Command staticCharacterization(double outputRampRate) {
    return pivot.staticCharacterization(outputRampRate);
  }

  // ==================== Direct Access (for testing/characterization) ====================

  public void runVoltsPivot(double volts) {
    pivot.runVolts(volts);
  }

  public void runVoltsRoller(double volts) {
    roller.runVolts(volts);
  }

  public void runOpenLoopPivot(double amps) {
    pivot.runOpenLoop(amps);
  }

  public void setPositionPivot(double degrees) {
    pivot.setPosition(degrees);
  }

  public void stopAll() {
    pivot.stop();
    roller.stop();
  }
}

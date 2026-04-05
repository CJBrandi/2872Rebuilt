// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  // Angle constants: 0° = ground/deployed, 90° = stowed
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(-10);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(90);
  public static final Rotation2d stowedAngle = Rotation2d.fromDegrees(90);
  public static final Rotation2d groundAngle = Rotation2d.fromDegrees(-8);

  private boolean runningIntake = false;
  private Supplier<Rotation2d> requestedPivotGoal = () -> groundAngle;
  private boolean trenchAutoDeployEnabled = false;

  // Subsystems
  @Getter private final Pivot pivot;
  @Getter private final Roller roller;

  public Intake(PivotIO pivotIO, RollerIO rollerIO) {
    this.pivot = new Pivot(pivotIO);
    this.roller = new Roller(rollerIO);
    pivot.setGoal(() -> getCommandedPivotGoal().getRadians());
  }

  public void setPivotGoal(Supplier<Rotation2d> goal) {
    requestedPivotGoal = goal;
  }

  public void setPivotGoal(DoubleSupplier goalRad) {
    requestedPivotGoal = () -> Rotation2d.fromRadians(goalRad.getAsDouble());
  }

  public void stow() {
    requestedPivotGoal = () -> stowedAngle;
    runningIntake = false;
    roller.stop();
  }

  public void deploy() {
    requestedPivotGoal = () -> groundAngle;
    runningIntake = true;
    roller.runIntake();
  }

  public void toggleIntake() {
    if (runningIntake) {
      roller.stop();
      runningIntake = false;
    } else {
      roller.runIntake();
      runningIntake = true;
    }
  }

  public boolean isAtGoal() {
    return pivot.isAtGoal();
  }

  public Rotation2d getPivotAngle() {
    return pivot.getAngle();
  }

  public Rotation2d getRequestedPivotGoal() {
    return requestedPivotGoal.get();
  }

  public Rotation2d getCommandedPivotGoal() {
    return trenchAutoDeployEnabled ? groundAngle : getRequestedPivotGoal();
  }

  public boolean requiresTrenchAutoDeploy() {
    return getRequestedPivotGoal().getDegrees() > Constants.IntakeBounds.maxDeployAngleDeg;
  }

  public boolean isPivotStowedForTrench() {
    return getPivotAngle().getDegrees() > Constants.IntakeBounds.maxDeployAngleDeg;
  }

  public double getWorstCaseDeployTimeSecs() {
    return pivot.getWorstCaseDeployTimeSecs();
  }

  public void setTrenchAutoDeployEnabled(boolean enabled) {
    trenchAutoDeployEnabled = enabled;
  }

  public boolean isTrenchAutoDeployEnabled() {
    return trenchAutoDeployEnabled;
  }

  /** Returns the homing sequence command for the pivot */
  public Command homingSequence() {
    return pivot.homingSequence();
  }

  public void runRollerVelocity(double velocityRPS) {
    roller.runVelocity(velocityRPS);
  }

  public Command staticCharacterization(double outputRampRate) {
    return pivot.staticCharacterization(outputRampRate);
  }

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

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/RequestedPivotGoalDeg", getRequestedPivotGoal().getDegrees());
    Logger.recordOutput("Intake/CommandedPivotGoalDeg", getCommandedPivotGoal().getDegrees());
    Logger.recordOutput("Intake/TrenchAutoDeployEnabled", trenchAutoDeployEnabled);
  }
}

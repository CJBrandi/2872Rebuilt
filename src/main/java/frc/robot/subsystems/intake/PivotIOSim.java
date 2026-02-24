// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {
  // Fourbar mechanism constants
  // Gear train: Motor → 22t gear → 30t gear → 12t sprocket → 22t sprocket → 23:1 cycloidal
  // Total reduction = (30/22) × (22/12) × 23 = 57.5:1
  public static final double reduction = PivotIOTalonFX.reduction; // 57.5:1

  // Fourbar arm physical properties
  private static final double armLengthMeters = Units.inchesToMeters(15.7);
  private static final double armMassKg = Units.lbsToKilograms(8.0);
  // Moment of inertia for a rod rotating about one end: I = (1/3) * m * L^2
  public static final double moi = (1.0 / 3.0) * armMassKg * armLengthMeters * armLengthMeters;
  // Center of gravity radius (distance from pivot to CG, typically L/2 for uniform arm)
  private static final double cgRadius = armLengthMeters / 2.0;

  // Gravity constant
  private static final double G = 9.81;

  // Motor model with full gear reduction
  public static final DCMotor gearbox = DCMotor.getKrakenX44(1).withReduction(reduction);

  // State-space matrices for the pivot dynamics
  // State: [position (rad), velocity (rad/s)]
  // Input: torque current (A)
  public static final Matrix<N2, N2> A =
      MatBuilder.fill(
          Nat.N2(),
          Nat.N2(),
          0,
          1,
          0,
          -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * moi));
  public static final Vector<N2> B = VecBuilder.fill(0, gearbox.KtNMPerAmp / moi);

  // State given by pivot angle position and velocity
  // Input given by torque current to motor
  private Vector<N2> simState;
  private double inputTorqueCurrent = 0.0;
  private double pivotAppliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private double feedforward = 0.0;
  private boolean closedLoop = false;

  public PivotIOSim() {
    // Initialize at stowed position (90 degrees)
    simState = VecBuilder.fill(Intake.stowedAngle.getRadians(), 0.0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      update(Constants.loopPeriodSecs);
    } else {
      // Run control at 1kHz for better accuracy
      for (int i = 0; i < Constants.loopPeriodSecs / (1.0 / 1000.0); i++) {
        setInputTorqueCurrent(controller.calculate(simState.get(0)) + feedforward);
        update(1.0 / 1000.0);
      }
    }
    // Pivot outputs
    inputs.internalPosition = Rotation2d.fromRadians(simState.get(0));
    inputs.encoderAbsolutePosition = Rotation2d.fromRadians(simState.get(0));
    inputs.velocityRadPerSec = simState.get(1);
    inputs.appliedVolts = pivotAppliedVolts;
    inputs.currentAmps = inputTorqueCurrent;
    inputs.motorConnected = true;
    inputs.encoderConnected = true;
  }

  @Override
  public void runOpenLoop(double output) {
    closedLoop = false;
    setInputTorqueCurrent(output);
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
    setInputVoltage(volts);
  }

  @Override
  public void stop() {
    runOpenLoop(0.0);
  }

  @Override
  public void runPosition(Rotation2d position, double feedforward) {
    closedLoop = true;
    controller.setSetpoint(position.getRadians());
    this.feedforward = feedforward;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }

  @Override
  public void setPosition(double degrees) {
    simState = VecBuilder.fill(Units.degreesToRadians(degrees), 0.0);
  }

  @Override
  public void homeToStowed() {
    setPosition(90.0);
  }

  private void setInputTorqueCurrent(double torqueCurrent) {
    inputTorqueCurrent = torqueCurrent;
    pivotAppliedVolts =
        gearbox.getVoltage(gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0));
    pivotAppliedVolts = MathUtil.clamp(pivotAppliedVolts, -12.0, 12.0);
  }

  private void setInputVoltage(double voltage) {
    pivotAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    inputTorqueCurrent = gearbox.getCurrent(simState.get(1, 0), pivotAppliedVolts);
  }

  private void update(double dt) {
    inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -40.0, 40.0);

    // Current pivot angle (0° = ground/horizontal, 90° = stowed/vertical)
    double currentAngleRad = simState.get(0);

    Matrix<N2, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> u) -> {
              Matrix<N2, N1> xdot = A.times(x).plus(B.times(u));

              // Add gravity torque
              // When angle = 0° (horizontal/ground), gravity torque is maximum
              // When angle = 90° (vertical/stowed), gravity torque is zero
              // Gravity torque = -m * g * r_cg * cos(angle)
              // The cos gives maximum torque at 0° and zero at 90°
              double gravityTorque = -armMassKg * G * cgRadius * Math.cos(x.get(0, 0));
              double gravityAccel = gravityTorque / moi;

              // Add gravity contribution to velocity derivative
              xdot.set(1, 0, xdot.get(1, 0) + gravityAccel);

              return xdot;
            },
            simState,
            VecBuilder.fill(inputTorqueCurrent),
            dt);

    // Apply limits
    simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));

    // Enforce angle limits with hard stops
    if (simState.get(0) <= Intake.minAngle.getRadians()) {
      simState.set(1, 0, Math.max(0.0, simState.get(1, 0))); // Only allow positive velocity
      simState.set(0, 0, Intake.minAngle.getRadians());
    }
    if (simState.get(0) >= Intake.maxAngle.getRadians()) {
      simState.set(1, 0, Math.min(0.0, simState.get(1, 0))); // Only allow negative velocity
      simState.set(0, 0, Intake.maxAngle.getRadians());
    }
  }
}

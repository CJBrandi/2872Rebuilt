// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class RollerIOSim implements RollerIO {
  private final DCMotorSim sim;
  private final DCMotor gearbox;
  private double appliedVoltage = 0.0;

  // Velocity control
  private final PIDController velocityController = new PIDController(0.5, 0, 0);
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0.12);
  private boolean velocityMode = false;
  private double velocitySetpointRPS = 0.0;

  public RollerIOSim(DCMotor motorModel, double reduction, double moi) {
    gearbox = motorModel;
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      velocityMode = false;
      runVolts(0.0);
    }

    // Run velocity control loop if in velocity mode
    if (velocityMode) {
      double currentVelocityRPS = sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
      double pidOutput = velocityController.calculate(currentVelocityRPS, velocitySetpointRPS);
      double ffOutput = feedforward.calculate(velocitySetpointRPS);
      appliedVoltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
      sim.setInputVoltage(appliedVoltage);
    }

    inputs.talonConnected = true;
    sim.update(Constants.loopPeriodSecs);
    inputs.talonPositionRads = sim.getAngularPositionRad();
    inputs.talonVelocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.talonAppliedVoltage = appliedVoltage;
    inputs.talonSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.talonTorqueCurrentAmps =
        gearbox.getCurrent(sim.getAngularVelocityRadPerSec(), appliedVoltage);
  }

  @Override
  public void runVelocity(double velocityRPS) {
    velocityMode = true;
    velocitySetpointRPS = velocityRPS;
  }

  @Override
  public void runTorqueCurrent(double current) {
    velocityMode = false;
    runVolts(gearbox.getVoltage(gearbox.getTorque(current), sim.getAngularVelocityRadPerSec()));
  }

  @Override
  public void runVolts(double volts) {
    velocityMode = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    velocityMode = false;
    runVolts(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double kS, double kV) {
    velocityController.setPID(kP, kI, kD);
    feedforward = new SimpleMotorFeedforward(kS, kV);
  }
}

package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO {
  // Moment of inertia for flywheel (kg*m^2)
  private static final double MOI = 0.005;

  // REV Vortex (NEO Vortex) with step-up gearing (flywheel spins faster than motor)
  // stepUp = 2 means flywheel is 2x motor speed, so reduction ratio = 1/stepUp = 0.5
  private static final DCMotor GEARBOX =
      DCMotor.getNeoVortex(1)
          .withReduction(
              1.0 / Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants.stepUp);

  // State-space model for velocity only: dx/dt = A*x + B*u
  // For a flywheel: dω/dt = -Kt/(Kv*R*J) * ω + Kt/(R*J) * I
  // Where ω is angular velocity, I is torque current
  private static final Matrix<N1, N1> A =
      MatBuilder.fill(
          Nat.N1(),
          Nat.N1(),
          -GEARBOX.KtNMPerAmp / (GEARBOX.KvRadPerSecPerVolt * GEARBOX.rOhms * MOI));
  private static final Vector<N1> B = VecBuilder.fill(GEARBOX.KtNMPerAmp / MOI);

  // State: angular velocity (rad/s)
  private Vector<N1> simState;
  private double inputTorqueCurrent = 0.0;
  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private double kS = 0.0;
  private double kV = 0.0;
  private boolean closedLoop = false;

  public FlywheelIOSim() {
    simState = VecBuilder.fill(0.0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      update(Constants.loopPeriodSecs);
    } else {
      // Run control at 1kHz for more accurate simulation
      double setpoint = controller.getSetpoint();
      double feedforward = (setpoint > 0 ? kS : 0) + kV * setpoint;
      for (int i = 0; i < (int) (Constants.loopPeriodSecs / (1.0 / 1000.0)); i++) {
        // Use velocity (simState.get(0)) as the measurement for velocity control
        setInputTorqueCurrent(controller.calculate(simState.get(0)) + feedforward);
        update(1.0 / 1000.0);
      }
    }

    inputs.motorConnected = true;
    inputs.encoderConnected = true;
    inputs.velocityRadPerSec = simState.get(0);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(inputTorqueCurrent);
    inputs.tempCelsius = 0.0;
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
  public void runVelocity(double radsPerSec) {
    closedLoop = true;
    controller.setSetpoint(radsPerSec);
  }

  @Override
  public void setFF(double kS, double kV) {
    this.kS = kS;
    this.kV = kV;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    // No-op in simulation
  }

  private void setInputTorqueCurrent(double torqueCurrent) {
    inputTorqueCurrent = MathUtil.clamp(torqueCurrent, -40.0, 40.0);
    appliedVolts = GEARBOX.getVoltage(GEARBOX.getTorque(inputTorqueCurrent), simState.get(0));
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  private void setInputVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);
    setInputTorqueCurrent(GEARBOX.getCurrent(simState.get(0), voltage));
  }

  private void update(double dt) {
    Matrix<N1, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N1, N1> x, Matrix<N1, N1> u) -> A.times(x).plus(B.times(u)),
            simState,
            VecBuilder.fill(inputTorqueCurrent),
            dt);

    simState = VecBuilder.fill(updatedState.get(0, 0));

    // No position limits for a flywheel - it spins freely
    // Optionally clamp to reasonable velocity limits to prevent numerical issues
    double maxVelocity = GEARBOX.freeSpeedRadPerSec;
    if (Math.abs(simState.get(0)) > maxVelocity) {
      simState = VecBuilder.fill(Math.signum(simState.get(0)) * maxVelocity);
    }
  }
}

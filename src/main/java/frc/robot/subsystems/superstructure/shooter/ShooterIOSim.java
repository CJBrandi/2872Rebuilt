package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  // Moment of inertia for flywheel (kg*m^2)
  private static final double moi = 0.005;

  private static final DCMotor gearbox =
      DCMotor.getKrakenX60Foc(1).withReduction(Constants.ShooterConstants.stepUp);

  // State-space model for velocity only: dx/dt = A*x + B*u
  // For a flywheel: dω/dt = -Kt/(Kv*R*J) * ω + Kt/(R*J) * I
  // Where ω is angular velocity, I is torque current
  private static final Matrix<N1, N1> A =
      MatBuilder.fill(
          Nat.N1(),
          Nat.N1(),
          -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * moi));
  private static final Vector<N1> B = VecBuilder.fill(gearbox.KtNMPerAmp / moi);

  // State: angular velocity (rad/s)
  private Vector<N1> simState;
  private double inputTorqueCurrent = 0.0;
  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private double feedforward = 0.0;
  private boolean closedLoop = false;

  public ShooterIOSim() {
    simState = VecBuilder.fill(0.0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      update(Constants.loopPeriodSecs);
    } else {
      // Run control at 1kHz for more accurate simulation
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
  public void runVelocity(double radsPerSec, double feedforward) {
    closedLoop = true;
    controller.setSetpoint(radsPerSec);
    this.feedforward = feedforward;
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
    appliedVolts = gearbox.getVoltage(gearbox.getTorque(inputTorqueCurrent), simState.get(0));
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  private void setInputVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);
    setInputTorqueCurrent(gearbox.getCurrent(simState.get(0), voltage));
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
    double maxVelocity = gearbox.freeSpeedRadPerSec;
    if (Math.abs(simState.get(0)) > maxVelocity) {
      simState = VecBuilder.fill(Math.signum(simState.get(0)) * maxVelocity);
    }
  }
}

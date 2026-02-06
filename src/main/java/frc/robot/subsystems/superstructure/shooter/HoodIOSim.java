package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public class HoodIOSim implements HoodIO {
  // Moment of inertia for hood mechanism (kg*m^2)
  private static final double MOI = 0.01;

  // Hood angle limits (radians)
  private static final double MIN_ANGLE_RAD = Math.toRadians(20.0); // 20 degrees
  private static final double MAX_ANGLE_RAD = Math.toRadians(70.0); // 70 degrees

  // Falcon 500 with reduction gearing
  private static final DCMotor GEARBOX =
      DCMotor.getFalcon500(1)
          .withReduction(
              Constants.SuperstructureConstants.ShooterConstants.HoodConstants.reduction);

  // State-space model: [position, velocity]
  // dx/dt = A*x + B*u
  private static final Matrix<N2, N2> A =
      MatBuilder.fill(
          Nat.N2(),
          Nat.N2(),
          0,
          1,
          0,
          -GEARBOX.KtNMPerAmp / (GEARBOX.KvRadPerSecPerVolt * GEARBOX.rOhms * MOI));
  private static final Vector<N2> B = VecBuilder.fill(0, GEARBOX.KtNMPerAmp / MOI);

  // State: [position (rad), velocity (rad/s)]
  private Vector<N2> simState;
  private double inputTorqueCurrent = 0.0;
  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private double feedforward = 0.0;
  private boolean closedLoop = false;

  public HoodIOSim() {
    // Start at minimum angle
    simState = VecBuilder.fill(MIN_ANGLE_RAD, 0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      update(Constants.loopPeriodSecs);
    } else {
      // Run control at 1kHz for more accurate simulation
      for (int i = 0; i < (int) (Constants.loopPeriodSecs / (1.0 / 1000.0)); i++) {
        setInputTorqueCurrent(controller.calculate(simState.get(0)) + feedforward);
        update(1.0 / 1000.0);
      }
    }

    inputs.motorConnected = true;
    inputs.encoderConnected = true;
    inputs.positionRad = simState.get(0);
    inputs.velocityRadPerSec = simState.get(1);
    inputs.appliedVolts = appliedVolts;
    inputs.torqueCurrentAmps = Math.abs(inputTorqueCurrent);
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void runOpenLoop(double output) {
    closedLoop = false;
    setInputTorqueCurrent(output);
  }

  @Override
  public void stop() {
    runOpenLoop(0.0);
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    closedLoop = true;
    controller.setSetpoint(positionRad);
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

  @Override
  public void setPosition(double radians) {
    simState = VecBuilder.fill(radians, 0.0);
  }

  private void setInputTorqueCurrent(double torqueCurrent) {
    inputTorqueCurrent = MathUtil.clamp(torqueCurrent, -40.0, 40.0);
    appliedVolts = GEARBOX.getVoltage(GEARBOX.getTorque(inputTorqueCurrent), simState.get(1));
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  private void setInputVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);
    setInputTorqueCurrent(GEARBOX.getCurrent(simState.get(1), voltage));
  }

  private void update(double dt) {
    Matrix<N2, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> u) -> A.times(x).plus(B.times(u)),
            simState,
            VecBuilder.fill(inputTorqueCurrent),
            dt);

    simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));

    // Apply hood angle limits with hard stops
    if (simState.get(0) <= MIN_ANGLE_RAD) {
      simState.set(0, 0, MIN_ANGLE_RAD);
      if (simState.get(1) < 0) {
        simState.set(1, 0, 0.0);
      }
    }
    if (simState.get(0) >= MAX_ANGLE_RAD) {
      simState.set(0, 0, MAX_ANGLE_RAD);
      if (simState.get(1) > 0) {
        simState.set(1, 0, 0.0);
      }
    }
  }
}

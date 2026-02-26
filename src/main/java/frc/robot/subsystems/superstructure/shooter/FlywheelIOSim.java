package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO {
  // Moment of inertia for flywheel (kg*m^2)
  private static final double MOI = 0.005;
  private static final int FLYWHEEL_MOTOR_COUNT = 2;
  private static final double MAX_TORQUE_CURRENT_PER_MOTOR_AMPS = 60.0;
  private static final double MAX_TORQUE_CURRENT_TOTAL_AMPS =
      MAX_TORQUE_CURRENT_PER_MOTOR_AMPS * FLYWHEEL_MOTOR_COUNT;

  // 2x Kraken X60 with step-up gearing (flywheel spins faster than motor)
  // stepUp = 2 means flywheel is 2x motor speed, so reduction ratio = 1/stepUp = 0.5
  private static final DCMotor GEARBOX =
      DCMotor.getKrakenX60(FLYWHEEL_MOTOR_COUNT)
          .withReduction(
              1.0 / Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants.stepUp);

  // State-space model for velocity only: dx/dt = A*x + B*u
  // Voltage-driven flywheel model:
  // dω/dt = -Kt/(Kv*R*J) * ω + Kt/(R*J) * V
  // Where ω is angular velocity and V is applied voltage.
  private static final Matrix<N1, N1> A =
      MatBuilder.fill(
          Nat.N1(),
          Nat.N1(),
          -GEARBOX.KtNMPerAmp / (GEARBOX.KvRadPerSecPerVolt * GEARBOX.rOhms * MOI));
  private static final Vector<N1> B = VecBuilder.fill(GEARBOX.KtNMPerAmp / (GEARBOX.rOhms * MOI));

  // State: angular velocity (rad/s)
  private Vector<N1> simState;
  private double commandedTorqueCurrent = 0.0;
  private double actualTorqueCurrent = 0.0;
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
      // Keep FF units consistent with real Talon velocity loop:
      // kV is in Amps per rotation/sec, while setpoint here is rad/sec.
      double feedforward = (setpoint > 0 ? kS : 0) + kV * Units.radiansToRotations(setpoint);
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
    inputs.currentAmps = Math.abs(actualTorqueCurrent);
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
    commandedTorqueCurrent =
        MathUtil.clamp(
            torqueCurrent, -MAX_TORQUE_CURRENT_TOTAL_AMPS, MAX_TORQUE_CURRENT_TOTAL_AMPS);
    appliedVolts = GEARBOX.getVoltage(GEARBOX.getTorque(commandedTorqueCurrent), simState.get(0));
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    actualTorqueCurrent = GEARBOX.getCurrent(simState.get(0), appliedVolts);
  }

  private void setInputVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);
    appliedVolts = voltage;
    actualTorqueCurrent = GEARBOX.getCurrent(simState.get(0), appliedVolts);
    commandedTorqueCurrent = actualTorqueCurrent;
  }

  private void update(double dt) {
    Matrix<N1, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N1, N1> x, Matrix<N1, N1> u) -> A.times(x).plus(B.times(u)),
            simState,
            VecBuilder.fill(appliedVolts),
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

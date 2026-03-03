package frc.robot.subsystems.superstructure.indexer;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IndexerIOSim implements IndexerIO {
  private static final double MOI = 0.003;
  private static final double RADIANS_PER_ROTATION = Units.rotationsToRadians(1.0);
  private static final double AUX_INDEXER_RUN_VOLTS = 6.0;
  private static final double AUX_INDEXER_RUN_EPSILON = 1e-3;

  private static final DCMotor GEARBOX =
      DCMotor.getKrakenX60Foc(1)
          .withReduction(Constants.SuperstructureConstants.IndexerConstants.reduction);

  private static final Matrix<N1, N1> A =
      MatBuilder.fill(
          Nat.N1(),
          Nat.N1(),
          -GEARBOX.KtNMPerAmp / (GEARBOX.KvRadPerSecPerVolt * GEARBOX.rOhms * MOI));
  private static final Vector<N1> B = VecBuilder.fill(GEARBOX.KtNMPerAmp / MOI);

  private Vector<N1> simState;
  private double inputTorqueCurrent = 0.0;
  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private double kS = 0.0;
  private double kVPerRotationPerSec = 0.0;
  private boolean closedLoop = false;
  private boolean auxIndexerRunning = false;

  public IndexerIOSim() {
    simState = VecBuilder.fill(0.0);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      update(Constants.loopPeriodSecs);
    } else {
      double setpoint = controller.getSetpoint();
      // Keep FF units consistent with Talon velocity loop:
      // kV is tuned in amps per rotation/sec, while setpoint here is rad/sec.
      double feedforward =
          kS * Math.signum(setpoint) + kVPerRotationPerSec * Units.radiansToRotations(setpoint);
      for (int i = 0; i < (int) (Constants.loopPeriodSecs / (1.0 / 1000.0)); i++) {
        setInputTorqueCurrent(controller.calculate(simState.get(0)) + feedforward);
        update(1.0 / 1000.0);
      }
    }

    inputs.motorConnected = true;
    inputs.followerConnected = true;
    inputs.encoderConnected = true;
    inputs.velocityRadPerSec = simState.get(0);
    inputs.appliedVolts =
        new double[] {appliedVolts, auxIndexerRunning ? AUX_INDEXER_RUN_VOLTS : 0.0};
    inputs.currentAmps = new double[] {Math.abs(inputTorqueCurrent), 0.0};
    inputs.tempCelsius = new double[] {0.0, 0.0};
  }

  @Override
  public void runOpenLoop(double output) {
    closedLoop = false;
    auxIndexerRunning = Math.abs(output) > AUX_INDEXER_RUN_EPSILON;
    setInputTorqueCurrent(output);
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
    auxIndexerRunning = Math.abs(volts) > AUX_INDEXER_RUN_EPSILON;
    setInputVoltage(volts);
  }

  @Override
  public void stop() {
    auxIndexerRunning = false;
    runOpenLoop(0.0);
  }

  @Override
  public void runVelocity(double radsPerSec) {
    closedLoop = true;
    auxIndexerRunning = Math.abs(radsPerSec) > AUX_INDEXER_RUN_EPSILON;
    controller.setSetpoint(radsPerSec);
  }

  @Override
  public void setFF(double kS, double kV) {
    this.kS = kS;
    this.kVPerRotationPerSec = kV;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    // Talon velocity gains are per rotation/sec. Convert to per rad/sec for the sim controller.
    controller.setPID(
        kP / RADIANS_PER_ROTATION, kI / RADIANS_PER_ROTATION, kD / RADIANS_PER_ROTATION);
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

    double maxVelocity = GEARBOX.freeSpeedRadPerSec;
    if (Math.abs(simState.get(0)) > maxVelocity) {
      simState = VecBuilder.fill(Math.signum(simState.get(0)) * maxVelocity);
    }
  }
}

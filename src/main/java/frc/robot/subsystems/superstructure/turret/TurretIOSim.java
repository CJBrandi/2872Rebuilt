package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class TurretIOSim implements TurretIO {
  private static final double moi = 0.001;

  // Turret hard stop limits (degrees)
  private static final double MAX_ANGLE_DEG = 185.0;

  // Hall effect sensor positions (degrees) and detection tolerance
  private static final double HALL_TOLERANCE_DEG = 2.0;

  public static final DCMotor gearbox =
      DCMotor.getKrakenX60Foc(1)
          .withReduction(Constants.SuperstructureConstants.TurretConstants.reduction);
  public static final Matrix<N2, N2> A =
      MatBuilder.fill(
          Nat.N2(),
          Nat.N2(),
          0,
          1,
          0,
          -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * moi));
  public static final Vector<N2> B = VecBuilder.fill(0, gearbox.KtNMPerAmp / moi);

  private Vector<N2> simState;
  private double inputTorqueCurrent = 0.0;
  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private double feedforward = 0.0;
  private boolean closedLoop = false;

  public TurretIOSim() {
    simState = VecBuilder.fill(0.0, 0.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      update(Constants.loopPeriodSecs);
    } else {
      // Run control at 1kHz
      for (int i = 0; i < Constants.loopPeriodSecs / (1.0 / 1000.0); i++) {
        setInputTorqueCurrent(controller.calculate(simState.get(0)) + feedforward);
        update(1.0 / 1000.0);
      }
    }

    inputs.motorConnected = true;
    inputs.encoderConnected = true;
    inputs.motorEncoderPosition = Rotation2d.fromRadians(simState.get(0));
    inputs.velocityRadPerSec = simState.get(1);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(inputTorqueCurrent);
    inputs.tempCelsius = 0.0;

    // Simulate hall effect sensors
    double positionDeg = Units.radiansToDegrees(simState.get(0));
    inputs.hallEffectState[0] =
        Math.abs(
                positionDeg
                    - Constants.SuperstructureConstants.TurretConstants.HallEffectDegrees.leftHall)
            < HALL_TOLERANCE_DEG;
    inputs.hallEffectState[1] =
        Math.abs(
                positionDeg
                    - Constants.SuperstructureConstants.TurretConstants.HallEffectDegrees
                        .middleHall)
            < HALL_TOLERANCE_DEG;
    inputs.hallEffectState[2] =
        Math.abs(
                positionDeg
                    - Constants.SuperstructureConstants.TurretConstants.HallEffectDegrees.rightHall)
            < HALL_TOLERANCE_DEG;
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
  public void runCurrent(double currentAmps) {
    closedLoop = false;
    setInputTorqueCurrent(currentAmps);
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
    simState = VecBuilder.fill(Math.toRadians(degrees), 0.0);
  }

  private void setInputTorqueCurrent(double torqueCurrent) {
    inputTorqueCurrent = MathUtil.clamp(torqueCurrent, -40.0, 40.0);
    appliedVolts = gearbox.getVoltage(gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0));
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  private void setInputVoltage(double voltage) {
    setInputTorqueCurrent(gearbox.getCurrent(simState.get(1, 0), voltage));
  }

  private void update(double dt) {
    Matrix<N2, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> u) -> A.times(x).plus(B.times(u)),
            simState,
            VecBuilder.fill(inputTorqueCurrent),
            dt);

    simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));

    // Apply turret hard stop limits
    double maxAngleRad = Units.degreesToRadians(MAX_ANGLE_DEG);
    if (simState.get(0) <= -maxAngleRad) {
      simState.set(0, 0, -maxAngleRad);
      if (simState.get(1) < 0) {
        simState.set(1, 0, 0.0);
      }
    }
    if (simState.get(0) >= maxAngleRad) {
      simState.set(0, 0, maxAngleRad);
      if (simState.get(1) > 0) {
        simState.set(1, 0, 0.0);
      }
    }
  }
}

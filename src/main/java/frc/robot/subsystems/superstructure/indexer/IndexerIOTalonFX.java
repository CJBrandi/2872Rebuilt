package frc.robot.subsystems.superstructure.indexer;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import java.util.function.Supplier;

/** Indexer IO implementation using TalonFX (Kraken X60). */
public class IndexerIOTalonFX implements IndexerIO {
  private static final double REDUCTION =
      Constants.SuperstructureConstants.IndexerConstants.reduction;
  private static final double AUX_INDEXER_RUN_VOLTS = 4.0;
  private static final double AUX_INDEXER_RUN_EPSILON = 1e-3;

  // Hardware
  private final TalonFX talon;
  private final SparkFlex indexer;
  private final RelativeEncoder encoder;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status signals
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

  // Control requests
  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public IndexerIOTalonFX(int canId) {
    this(canId, "");
  }

  public IndexerIOTalonFX(int canId, String canBus) {
    talon = new TalonFX(canId, canBus);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0 = new Slot0Configs().withKS(0.0).withKV(0.0).withKP(0.0).withKI(0.0).withKD(0.0);
    config.Feedback.SensorToMechanismRatio = REDUCTION;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    torqueCurrent = talon.getTorqueCurrent();
    supplyCurrent = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp);
    ParentDevice.optimizeBusUtilizationForAll(talon);

    indexer =
        new SparkFlex(
            Constants.SuperstructureConstants.IndexerConstants.vortexId,
            SparkLowLevel.MotorType.kBrushless);
    encoder = indexer.getEncoder();
    SparkFlexConfig indexerConfig = new SparkFlexConfig();
    indexerConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(false).smartCurrentLimit(40);
    indexerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20)
        .motorTemperaturePeriodMs(20);
    tryUntilOkRev(
        5,
        () ->
            indexer.configure(
                indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOkRev(5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    boolean talonConnected =
        BaseStatusSignal.refreshAll(velocity, appliedVolts, torqueCurrent, supplyCurrent, temp)
            .isOK();
    double leaderAppliedVolts = appliedVolts.getValueAsDouble();
    double leaderCurrentAmps = torqueCurrent.getValueAsDouble();
    double leaderTempCelsius = temp.getValueAsDouble();

    double auxBusVolts = indexer.getBusVoltage();
    REVLibError auxBusVoltsError = indexer.getLastError();
    double auxAppliedOutput = indexer.getAppliedOutput();
    REVLibError auxAppliedOutputError = indexer.getLastError();
    double auxAppliedVolts = auxBusVolts * auxAppliedOutput;
    double auxCurrentAmps = indexer.getOutputCurrent();
    REVLibError auxCurrentError = indexer.getLastError();
    double auxTempCelsius = indexer.getMotorTemperature();
    REVLibError auxTempError = indexer.getLastError();
    boolean auxConnected =
        auxBusVoltsError == REVLibError.kOk
            && auxAppliedOutputError == REVLibError.kOk
            && auxCurrentError == REVLibError.kOk
            && auxTempError == REVLibError.kOk;

    inputs.motorConnected = motorConnectedDebouncer.calculate(talonConnected);
    inputs.followerConnected = followerConnectedDebouncer.calculate(auxConnected);
    inputs.encoderConnected = encoderConnectedDebouncer.calculate(talonConnected);
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = new double[] {leaderAppliedVolts, auxAppliedVolts};
    inputs.currentAmps = new double[] {leaderCurrentAmps, auxCurrentAmps};
    inputs.tempCelsius = new double[] {leaderTempCelsius, auxTempCelsius};
  }

  @Override
  public void runOpenLoop(double output) {
    talon.setControl(torqueCurrentRequest.withOutput(output));
    runAuxIndexer(Math.abs(output) > AUX_INDEXER_RUN_EPSILON);
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
    runAuxIndexer(Math.abs(volts) > AUX_INDEXER_RUN_EPSILON);
  }

  @Override
  public void stop() {
    talon.stopMotor();
    runAuxIndexer(false);
  }

  @Override
  public void runVelocity(double radsPerSec) {
    talon.setControl(
        velocityTorqueCurrentRequest.withVelocity(Units.radiansToRotations(radsPerSec)));
    runAuxIndexer(Math.abs(radsPerSec) > AUX_INDEXER_RUN_EPSILON);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setFF(double kS, double kV) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast))
        .start();
  }

  private void runAuxIndexer(boolean talonRunning) {
    indexer.setVoltage(talonRunning ? AUX_INDEXER_RUN_VOLTS : 0.0);
  }

  private static void tryUntilOkRev(int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get() == REVLibError.kOk) {
        break;
      }
    }
  }
}

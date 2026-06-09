package frc.robot.subsystems.superstructure.indexer;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Indexer {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Superstructure/Indexer/kP");
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Superstructure/Indexer/kI");
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Superstructure/Indexer/kD");
  // Talon velocity-loop units:
  // kP/kI/kD are amps per (mechanism rotation/sec) error terms.
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Superstructure/Indexer/kS");
  // kV is amps per mechanism rotation/sec.
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Superstructure/Indexer/kV");

  private static final LoggedTunableNumber maxVelocityRadPerSec =
      new LoggedTunableNumber("Superstructure/Indexer/MaxVelocityRadPerSec");
  private static final LoggedTunableNumber maxAccelerationRadPerSec2 =
      new LoggedTunableNumber("Superstructure/Indexer/MaxAccelerationRadPerSec2");
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber(
          "Superstructure/Indexer/StaticCharacterizationVelocityThreshRadPerSec");
  private static final LoggedTunableNumber manualModeEnabled =
      new LoggedTunableNumber("Manual/Enabled", 0.0);
  private static final LoggedTunableNumber manualIndexerRPM =
      new LoggedTunableNumber("Manual/IndexerRPM", 0.0);
  private static final LoggedTunableNumber intakeRPM =
      new LoggedTunableNumber("Superstructure/Indexer/IntakeRPM");
  private static final LoggedTunableNumber jamDetectionCurrentAmps =
      new LoggedTunableNumber("Superstructure/Indexer/Unjam/DetectionCurrentAmps");
  private static final LoggedTunableNumber jamDetectionVelocityRPM =
      new LoggedTunableNumber("Superstructure/Indexer/Unjam/DetectionVelocityRPM");
  private static final LoggedTunableNumber jamDetectionCommandVelocityRPM =
      new LoggedTunableNumber("Superstructure/Indexer/Unjam/DetectionCommandVelocityRPM");
  private static final LoggedTunableNumber jamDetectionTimeSecs =
      new LoggedTunableNumber("Superstructure/Indexer/Unjam/DetectionTimeSecs");
  private static final LoggedTunableNumber unjamReverseCurrentAmps =
      new LoggedTunableNumber("Superstructure/Indexer/Unjam/ReverseCurrentAmps");
  private static final LoggedTunableNumber unjamForwardCurrentAmps =
      new LoggedTunableNumber("Superstructure/Indexer/Unjam/ForwardCurrentAmps");
  private static final LoggedTunableNumber unjamReverseTimeSecs =
      new LoggedTunableNumber("Superstructure/Indexer/Unjam/ReverseTimeSecs");
  private static final LoggedTunableNumber unjamForwardTimeSecs =
      new LoggedTunableNumber("Superstructure/Indexer/Unjam/ForwardTimeSecs");

  static {
    switch (Constants.getCurrentMode()) {
      case REAL -> {
        kP.initDefault(80);
        kI.initDefault(0.0);
        kD.initDefault(0.0);
        kS.initDefault(2.85);
        kV.initDefault(2.5);
        maxVelocityRadPerSec.initDefault(200.0);
        maxAccelerationRadPerSec2.initDefault(400.0);
        staticCharacterizationVelocityThresh.initDefault(0.1);
        jamDetectionCurrentAmps.initDefault(30.0);
        jamDetectionVelocityRPM.initDefault(5.0);
        jamDetectionCommandVelocityRPM.initDefault(40.0);
        jamDetectionTimeSecs.initDefault(0.15);
        unjamReverseCurrentAmps.initDefault(40.0);
        unjamForwardCurrentAmps.initDefault(40.0);
        unjamReverseTimeSecs.initDefault(0.1);
        unjamForwardTimeSecs.initDefault(0.08);
      }
      case SIM, REPLAY -> {
        kP.initDefault(6.0);
        kI.initDefault(0.0);
        kD.initDefault(0.0);
        kS.initDefault(0.0);
        kV.initDefault(0.0);
        maxVelocityRadPerSec.initDefault(300.0);
        maxAccelerationRadPerSec2.initDefault(600.0);
        staticCharacterizationVelocityThresh.initDefault(0.1);
        jamDetectionCurrentAmps.initDefault(30.0);
        jamDetectionVelocityRPM.initDefault(5.0);
        jamDetectionCommandVelocityRPM.initDefault(40.0);
        jamDetectionTimeSecs.initDefault(0.15);
        unjamReverseCurrentAmps.initDefault(20.0);
        unjamForwardCurrentAmps.initDefault(28.0);
        unjamReverseTimeSecs.initDefault(0.08);
        unjamForwardTimeSecs.initDefault(0.06);
      }
    }
    intakeRPM.initDefault(110.0);
  }

  private enum AutoUnjamState {
    IDLE,
    REVERSE_PULSE,
    FORWARD_PULSE
  }

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private TrapezoidProfile profile;
  @Getter private TrapezoidProfile.State setpoint;
  private double targetPositionRad = 0.0;

  @Getter private double targetVelocityRadPerSec = 0.0;
  private boolean closedLoop = false;
  private boolean staticCharacterizationActive = false;
  private boolean wasManualMode = false;
  private boolean profileNeedsReset = false;
  private AutoUnjamState autoUnjamState = AutoUnjamState.IDLE;
  private double jamDetectionElapsedSecs = 0.0;
  private double autoUnjamStateElapsedSecs = 0.0;
  private double autoUnjamTotalElapsedSecs = 0.0;

  @Getter private boolean atSetpoint = false;

  public Indexer(IndexerIO io) {
    this.io = io;
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityRadPerSec.get(), maxAccelerationRadPerSec2.get()));
    setpoint = new TrapezoidProfile.State(0.0, 0.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Indexer", inputs);

    boolean manualMode = manualModeEnabled.get() > 0.5;
    if (manualMode && !staticCharacterizationActive) {
      runVelocityRPM(manualIndexerRPM.get());
    } else if (wasManualMode && !staticCharacterizationActive) {
      // Reset to zero once when exiting manual mode.
      stop();
    }
    wasManualMode = manualMode;

    // Update gains/config when tunables change
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(hashCode(), () -> io.setFF(kS.get(), kV.get()), kS, kV);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            profile =
                new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                        maxVelocityRadPerSec.get(), maxAccelerationRadPerSec2.get())),
        maxVelocityRadPerSec,
        maxAccelerationRadPerSec2);

    if (closedLoop) {
      updateAutoUnjamState();
      if (autoUnjamState == AutoUnjamState.IDLE) {
        if (profileNeedsReset) {
          seedProfileFromMeasuredVelocity();
        }

        // Move a virtual position target at the desired steady-state velocity and profile towards
        // it.
        targetPositionRad += targetVelocityRadPerSec * Constants.loopPeriodSecs;
        var goalState = new TrapezoidProfile.State(targetPositionRad, targetVelocityRadPerSec);
        setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
        io.runVelocity(setpoint.velocity);

        atSetpoint =
            EqualsUtil.epsilonEquals(
                inputs.velocityRadPerSec,
                setpoint.velocity,
                Units.rotationsPerMinuteToRadiansPerSecond(50.0));
      } else {
        runAutoUnjamOutput();
        atSetpoint = false;
      }
    } else {
      resetAutoUnjamState(false);
      atSetpoint = false;
    }

    Logger.recordOutput(
        "Superstructure/Indexer/TargetVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(targetVelocityRadPerSec));
    Logger.recordOutput(
        "Superstructure/Indexer/SetpointVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(setpoint.velocity));
    Logger.recordOutput(
        "Superstructure/Indexer/MeasuredVelocityRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec));
    Logger.recordOutput("Superstructure/Indexer/AuxMeasuredVelocityRPM", inputs.auxVelocityRPM);
    Logger.recordOutput("Superstructure/Indexer/AtSetpoint", atSetpoint);
    Logger.recordOutput("Superstructure/Indexer/ClosedLoop", closedLoop);
    Logger.recordOutput(
        "Superstructure/Indexer/StaticCharacterizationActive", staticCharacterizationActive);
    Logger.recordOutput("Superstructure/Indexer/ManualMode", manualMode);
    Logger.recordOutput("Superstructure/Indexer/Manual/TargetRPM", manualIndexerRPM.get());
    Logger.recordOutput("Superstructure/Indexer/IntakeTargetRPM", intakeRPM.get());
    Logger.recordOutput("Superstructure/Indexer/Unjam/State", autoUnjamState.name());
    Logger.recordOutput(
        "Superstructure/Indexer/Unjam/DetectionElapsedSecs", jamDetectionElapsedSecs);
    Logger.recordOutput("Superstructure/Indexer/Unjam/StateElapsedSecs", autoUnjamStateElapsedSecs);
    Logger.recordOutput("Superstructure/Indexer/Unjam/TotalElapsedSecs", autoUnjamTotalElapsedSecs);
    Logger.recordOutput(
        "Superstructure/Indexer/Unjam/Stalled",
        autoUnjamState == AutoUnjamState.IDLE && isStalled());
  }

  /** Runs closed-loop velocity (rad/s). */
  public void runVelocity(double velocityRadPerSec) {
    if (!closedLoop) {
      // Seed profile state from measured velocity for bumpless transfer.
      setpoint = new TrapezoidProfile.State(0.0, inputs.velocityRadPerSec);
      targetPositionRad = setpoint.position;
    }

    closedLoop = true;
    targetVelocityRadPerSec = velocityRadPerSec;
  }

  /** Runs closed-loop velocity (RPM). */
  public void runVelocityRPM(double velocityRPM) {
    runVelocity(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM));
  }

  /** Runs the configured superstructure intake/feed RPM setpoint. */
  public void runIntakeVelocity() {
    runVelocityRPM(intakeRPM.get());
  }

  public void runVolts(double volts) {
    closedLoop = false;
    targetVelocityRadPerSec = 0.0;
    resetAutoUnjamState(false);
    io.runVolts(volts);
  }

  public void runOpenLoop(double output) {
    closedLoop = false;
    targetVelocityRadPerSec = 0.0;
    resetAutoUnjamState(false);
    io.runOpenLoop(output);
  }

  /** Decelerates to zero using the trapezoidal velocity profile. */
  public void stop() {
    runVelocity(0.0);
  }

  /** State class for static characterization. */
  private static class StaticCharacterizationState {
    public double characterizationCurrentAmps = 0.0;
  }

  /**
   * Creates a command for static characterization that ramps current until motion is detected.
   *
   * @param currentRampRateAmpsPerSec Rate at which to increase current (amps per second)
   * @return Command that runs the characterization
   */
  public Command staticCharacterization(double currentRampRateAmpsPerSec) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              staticCharacterizationActive = true;
              closedLoop = false;
              timer.restart();
            },
            () -> {
              // Keep closed-loop off while characterizing.
              closedLoop = false;
              state.characterizationCurrentAmps = currentRampRateAmpsPerSec * timer.get();
              runOpenLoop(state.characterizationCurrentAmps);
              Logger.recordOutput(
                  "Superstructure/Indexer/StaticCharacterizationCurrentAmps",
                  state.characterizationCurrentAmps);
            })
        .until(
            () -> Math.abs(inputs.velocityRadPerSec) >= staticCharacterizationVelocityThresh.get())
        .finallyDo(
            () -> {
              staticCharacterizationActive = false;
              timer.stop();
              io.stop();
              Logger.recordOutput(
                  "Superstructure/Indexer/CharacterizationResultCurrentAmps",
                  state.characterizationCurrentAmps);
            });
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public boolean isMotorConnected() {
    return inputs.motorConnected;
  }

  private void seedProfileFromMeasuredVelocity() {
    setpoint = new TrapezoidProfile.State(0.0, inputs.velocityRadPerSec);
    targetPositionRad = setpoint.position;
    profileNeedsReset = false;
  }

  private void updateAutoUnjamState() {
    if (!shouldEnableAutoUnjam()) {
      resetAutoUnjamState(autoUnjamState != AutoUnjamState.IDLE);
      return;
    }

    switch (autoUnjamState) {
      case IDLE -> {
        if (isStalled()) {
          jamDetectionElapsedSecs += Constants.loopPeriodSecs;
          if (jamDetectionElapsedSecs >= jamDetectionTimeSecs.get()) {
            autoUnjamState = AutoUnjamState.REVERSE_PULSE;
            jamDetectionElapsedSecs = 0.0;
            autoUnjamStateElapsedSecs = 0.0;
            autoUnjamTotalElapsedSecs = 0.0;
          }
        } else {
          jamDetectionElapsedSecs = 0.0;
        }
      }
      case REVERSE_PULSE, FORWARD_PULSE -> {
        jamDetectionElapsedSecs = 0.0;
        autoUnjamStateElapsedSecs += Constants.loopPeriodSecs;
        autoUnjamTotalElapsedSecs += Constants.loopPeriodSecs;

        if (autoUnjamState == AutoUnjamState.REVERSE_PULSE
            && autoUnjamStateElapsedSecs >= unjamReverseTimeSecs.get()) {
          autoUnjamState = AutoUnjamState.FORWARD_PULSE;
          autoUnjamStateElapsedSecs = 0.0;
        } else if (autoUnjamState == AutoUnjamState.FORWARD_PULSE) {
          if (isRecoveredFromJam()) {
            resetAutoUnjamState(true);
          } else if (autoUnjamStateElapsedSecs >= unjamForwardTimeSecs.get()) {
            autoUnjamState = AutoUnjamState.REVERSE_PULSE;
            autoUnjamStateElapsedSecs = 0.0;
          }
        }
      }
    }
  }

  private boolean shouldEnableAutoUnjam() {
    return inputs.motorConnected
        && !staticCharacterizationActive
        && targetVelocityRadPerSec
            >= Units.rotationsPerMinuteToRadiansPerSecond(jamDetectionCommandVelocityRPM.get());
  }

  private boolean isStalled() {
    return inputs.currentAmps >= jamDetectionCurrentAmps.get()
        && Math.abs(inputs.velocityRadPerSec)
            <= Units.rotationsPerMinuteToRadiansPerSecond(jamDetectionVelocityRPM.get());
  }

  private boolean isRecoveredFromJam() {
    return inputs.velocityRadPerSec
        >= Units.rotationsPerMinuteToRadiansPerSecond(jamDetectionVelocityRPM.get());
  }

  private void runAutoUnjamOutput() {
    switch (autoUnjamState) {
      case REVERSE_PULSE -> io.runOpenLoop(-unjamReverseCurrentAmps.get());
      case FORWARD_PULSE -> io.runOpenLoop(unjamForwardCurrentAmps.get());
      case IDLE -> {}
    }
  }

  private void resetAutoUnjamState(boolean resetProfile) {
    autoUnjamState = AutoUnjamState.IDLE;
    jamDetectionElapsedSecs = 0.0;
    autoUnjamStateElapsedSecs = 0.0;
    autoUnjamTotalElapsedSecs = 0.0;
    profileNeedsReset = profileNeedsReset || resetProfile;
  }
}

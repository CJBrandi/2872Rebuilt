package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class RollerTest {
  private static final double JAM_CURRENT_AMPS = 35.0;
  private static final double FREE_VELOCITY_RPS = 12.0;

  @Test
  void entersReversePulseAfterSustainedStall() {
    FakeRollerIO io = new FakeRollerIO();
    Roller roller = new Roller(io);

    for (int i = 0; i < loopsFor(0.15); i++) {
      runIntakeLoop(roller, io, JAM_CURRENT_AMPS, 0.0);
    }

    assertEquals(FakeRollerIO.OutputMode.TORQUE_CURRENT, io.outputMode);
    assertTrue(io.lastTorqueCurrentAmps < 0.0);
  }

  @Test
  void resumesVelocityControlOnceForwardPulseRestoresMotion() {
    FakeRollerIO io = new FakeRollerIO();
    Roller roller = new Roller(io);

    for (int i = 0; i < loopsFor(0.15); i++) {
      runIntakeLoop(roller, io, JAM_CURRENT_AMPS, 0.0);
    }
    for (int i = 0; i < loopsFor(0.08); i++) {
      runIntakeLoop(roller, io, JAM_CURRENT_AMPS, 0.0);
    }

    runIntakeLoop(roller, io, 5.0, FREE_VELOCITY_RPS);

    assertEquals(FakeRollerIO.OutputMode.VELOCITY, io.outputMode);
    assertTrue(io.lastVelocitySetpointRPS > 0.0);
  }

  @Test
  void keepsRetryingWhileJamPersists() {
    FakeRollerIO io = new FakeRollerIO();
    Roller roller = new Roller(io);

    for (int i = 0; i < loopsFor(0.15); i++) {
      runIntakeLoop(roller, io, JAM_CURRENT_AMPS, 0.0);
    }
    for (int i = 0; i < loopsFor(0.75); i++) {
      runIntakeLoop(roller, io, JAM_CURRENT_AMPS, 0.0);
    }

    runIntakeLoop(roller, io, JAM_CURRENT_AMPS, 0.0);

    assertEquals(FakeRollerIO.OutputMode.TORQUE_CURRENT, io.outputMode);
    assertTrue(Math.abs(io.lastTorqueCurrentAmps) > 0.0);
  }

  @Test
  void doesNotUnjamForHoldVelocity() {
    FakeRollerIO io = new FakeRollerIO();
    Roller roller = new Roller(io);

    for (int i = 0; i < loopsFor(0.25); i++) {
      io.talonTorqueCurrentAmps = JAM_CURRENT_AMPS;
      io.talonVelocityRadsPerSec = 0.0;
      roller.runHold();
      roller.periodic();
    }

    assertEquals(FakeRollerIO.OutputMode.VELOCITY, io.outputMode);
    assertTrue(io.lastVelocitySetpointRPS >= 0.0);
  }

  @Test
  void temporaryVelocityOverrideRestoresPreviousVelocityOnRelease() {
    FakeRollerIO io = new FakeRollerIO();
    Roller roller = new Roller(io);

    roller.runIntake();
    roller.periodic();
    roller.setTemporaryVelocityOverride(-Roller.intakeVelocity.get());
    roller.periodic();

    assertTrue(roller.getTargetVelocityRPS() < 0.0);

    roller.clearTemporaryVelocityOverride();

    assertEquals(Roller.intakeVelocity.get(), roller.getTargetVelocityRPS());
  }

  @Test
  void temporaryVelocityOverrideResumesLatestRequestedCommand() {
    FakeRollerIO io = new FakeRollerIO();
    Roller roller = new Roller(io);

    roller.runIntake();
    roller.periodic();
    roller.setTemporaryVelocityOverride(-Roller.intakeVelocity.get());
    roller.periodic();
    roller.stop();
    roller.periodic();

    assertEquals(FakeRollerIO.OutputMode.VELOCITY, io.outputMode);
    assertTrue(io.lastVelocitySetpointRPS < 0.0);

    roller.clearTemporaryVelocityOverride();

    assertEquals(FakeRollerIO.OutputMode.STOPPED, io.outputMode);
  }

  private static int loopsFor(double seconds) {
    return (int) Math.ceil(seconds / Constants.loopPeriodSecs);
  }

  private static void runIntakeLoop(
      Roller roller, FakeRollerIO io, double torqueCurrentAmps, double velocityRPS) {
    io.talonTorqueCurrentAmps = torqueCurrentAmps;
    io.talonVelocityRadsPerSec = velocityRPS * 2.0 * Math.PI;
    roller.runIntake();
    roller.periodic();
  }

  private static class FakeRollerIO implements RollerIO {
    private enum OutputMode {
      VELOCITY,
      TORQUE_CURRENT,
      STOPPED
    }

    private double talonVelocityRadsPerSec = 0.0;
    private double talonTorqueCurrentAmps = 0.0;
    private double lastVelocitySetpointRPS = 0.0;
    private double lastTorqueCurrentAmps = 0.0;
    private OutputMode outputMode = OutputMode.STOPPED;

    @Override
    public void updateInputs(RollerIOInputs inputs) {
      inputs.talonConnected = true;
      inputs.talonPositionRads = 0.0;
      inputs.talonVelocityRadsPerSec = talonVelocityRadsPerSec;
      inputs.talonAppliedVoltage = 0.0;
      inputs.talonSupplyCurrentAmps = 0.0;
      inputs.talonTorqueCurrentAmps = talonTorqueCurrentAmps;
      inputs.talonTempCelsius = 0.0;
    }

    @Override
    public void runVelocity(double velocityRPS) {
      outputMode = OutputMode.VELOCITY;
      lastVelocitySetpointRPS = velocityRPS;
    }

    @Override
    public void runTorqueCurrent(double current) {
      outputMode = OutputMode.TORQUE_CURRENT;
      lastTorqueCurrentAmps = current;
    }

    @Override
    public void stop() {
      outputMode = OutputMode.STOPPED;
      lastVelocitySetpointRPS = 0.0;
      lastTorqueCurrentAmps = 0.0;
    }
  }
}

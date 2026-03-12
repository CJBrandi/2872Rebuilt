package frc.robot.subsystems.superstructure.indexer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class IndexerTest {
  private static final double JAM_CURRENT_AMPS = 35.0;
  private static final double FREE_VELOCITY_RAD_PER_SEC =
      Units.rotationsPerMinuteToRadiansPerSecond(20.0);

  @Test
  void entersReversePulseAfterSustainedStall() {
    FakeIndexerIO io = new FakeIndexerIO();
    Indexer indexer = new Indexer(io);

    for (int i = 0; i < loopsFor(0.15); i++) {
      runFeedLoop(indexer, io, JAM_CURRENT_AMPS, 0.0);
    }

    assertEquals(FakeIndexerIO.OutputMode.OPEN_LOOP, io.outputMode);
    assertTrue(io.lastOpenLoopOutput < 0.0);
  }

  @Test
  void resumesVelocityControlOnceForwardPulseRestoresMotion() {
    FakeIndexerIO io = new FakeIndexerIO();
    Indexer indexer = new Indexer(io);

    for (int i = 0; i < loopsFor(0.15); i++) {
      runFeedLoop(indexer, io, JAM_CURRENT_AMPS, 0.0);
    }
    for (int i = 0; i < loopsFor(0.08); i++) {
      runFeedLoop(indexer, io, JAM_CURRENT_AMPS, 0.0);
    }

    runFeedLoop(indexer, io, 5.0, FREE_VELOCITY_RAD_PER_SEC);

    assertEquals(FakeIndexerIO.OutputMode.VELOCITY, io.outputMode);
    assertTrue(io.lastVelocitySetpointRadPerSec > 0.0);
  }

  @Test
  void keepsRetryingWhileJamPersists() {
    FakeIndexerIO io = new FakeIndexerIO();
    Indexer indexer = new Indexer(io);

    for (int i = 0; i < loopsFor(0.15); i++) {
      runFeedLoop(indexer, io, JAM_CURRENT_AMPS, 0.0);
    }
    for (int i = 0; i < loopsFor(0.75); i++) {
      runFeedLoop(indexer, io, JAM_CURRENT_AMPS, 0.0);
    }

    runFeedLoop(indexer, io, JAM_CURRENT_AMPS, 0.0);
    assertEquals(FakeIndexerIO.OutputMode.OPEN_LOOP, io.outputMode);
    assertTrue(Math.abs(io.lastOpenLoopOutput) > 0.0);
  }

  private static int loopsFor(double seconds) {
    return (int) Math.ceil(seconds / Constants.loopPeriodSecs);
  }

  private static void runFeedLoop(
      Indexer indexer, FakeIndexerIO io, double currentAmps, double velocityRadPerSec) {
    io.currentAmps = currentAmps;
    io.velocityRadPerSec = velocityRadPerSec;
    indexer.runIntakeVelocity();
    indexer.periodic();
  }

  private static class FakeIndexerIO implements IndexerIO {
    private enum OutputMode {
      VELOCITY,
      OPEN_LOOP,
      STOPPED
    }

    private double velocityRadPerSec = 0.0;
    private double currentAmps = 0.0;
    private double lastVelocitySetpointRadPerSec = 0.0;
    private double lastOpenLoopOutput = 0.0;
    private OutputMode outputMode = OutputMode.STOPPED;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
      inputs.motorConnected = true;
      inputs.followerConnected = true;
      inputs.encoderConnected = true;
      inputs.velocityRadPerSec = velocityRadPerSec;
      inputs.auxVelocityRPM = 0.0;
      inputs.appliedVolts = 0.0;
      inputs.currentAmps = currentAmps;
      inputs.tempCelsius = 0.0;
    }

    @Override
    public void runOpenLoop(double output) {
      outputMode = OutputMode.OPEN_LOOP;
      lastOpenLoopOutput = output;
    }

    @Override
    public void runVelocity(double radsPerSec) {
      outputMode = OutputMode.VELOCITY;
      lastVelocitySetpointRadPerSec = radsPerSec;
    }

    @Override
    public void stop() {
      outputMode = OutputMode.STOPPED;
      lastOpenLoopOutput = 0.0;
      lastVelocitySetpointRadPerSec = 0.0;
    }
  }
}

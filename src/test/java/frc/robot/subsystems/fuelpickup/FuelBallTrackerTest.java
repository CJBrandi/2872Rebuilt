package frc.robot.subsystems.fuelpickup;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class FuelBallTrackerTest {
  @Test
  void keepsTrackIdStableAcrossFrames() {
    FuelBallTracker tracker = new FuelBallTracker(0.55, 0.25, 55.0, 70.0, 2, 8);

    tracker.update(List.of(new FuelBallTracker.Detection(100.0, 120.0, 12.0, 0.9)), 0.00);
    FuelBallTracker.TrackerOutput frameTwo =
        tracker.update(List.of(new FuelBallTracker.Detection(104.0, 123.0, 11.5, 0.92)), 0.02);

    assertEquals(1, frameTwo.confirmedTracks().size());
    int confirmedId = frameTwo.confirmedTracks().get(0).id();
    assertTrue(confirmedId > 0);

    FuelBallTracker.TrackerOutput frameThree =
        tracker.update(List.of(new FuelBallTracker.Detection(108.0, 126.0, 11.0, 0.91)), 0.04);
    assertEquals(1, frameThree.confirmedTracks().size());
    assertEquals(confirmedId, frameThree.confirmedTracks().get(0).id());
  }
}

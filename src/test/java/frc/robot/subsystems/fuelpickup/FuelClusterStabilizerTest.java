package frc.robot.subsystems.fuelpickup;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.Map;
import org.junit.jupiter.api.Test;

class FuelClusterStabilizerTest {
  @Test
  void keepsStableIdForNearbyCentroid() {
    FuelClusterStabilizer stabilizer = new FuelClusterStabilizer(1.2, 8, 0.45);

    Map<Integer, Integer> frameOne =
        stabilizer.update(Map.of(0, new Translation3d(2.0, 1.0, 0.075)));
    int stableId = frameOne.get(0);

    Map<Integer, Integer> frameTwo =
        stabilizer.update(Map.of(0, new Translation3d(2.3, 1.1, 0.075)));
    assertEquals(stableId, frameTwo.get(0));
  }

  @Test
  void assignsNewStableIdForFarCentroid() {
    FuelClusterStabilizer stabilizer = new FuelClusterStabilizer(1.2, 8, 0.45);

    int stableOne = stabilizer.update(Map.of(0, new Translation3d(2.0, 1.0, 0.075))).get(0);
    int stableTwo = stabilizer.update(Map.of(0, new Translation3d(7.0, 5.0, 0.075))).get(0);

    assertNotEquals(stableOne, stableTwo);
  }
}

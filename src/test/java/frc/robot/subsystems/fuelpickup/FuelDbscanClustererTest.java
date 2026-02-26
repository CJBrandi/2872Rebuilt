package frc.robot.subsystems.fuelpickup;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import org.junit.jupiter.api.Test;

class FuelDbscanClustererTest {
  @Test
  void separatesTwoClustersAndNoise() {
    FuelDbscanClusterer clusterer = new FuelDbscanClusterer(0.8, 2);
    List<Translation3d> points =
        List.of(
            new Translation3d(1.0, 1.0, 0.075),
            new Translation3d(1.4, 1.2, 0.075),
            new Translation3d(5.0, 4.7, 0.075),
            new Translation3d(5.2, 4.5, 0.075),
            new Translation3d(9.5, 0.5, 0.075));

    FuelDbscanClusterer.ClusterResult result = clusterer.cluster(points);

    assertEquals(2, result.clusterCount());
    int[] labels = result.labels();
    assertTrue(labels[0] >= 0);
    assertEquals(labels[0], labels[1]);
    assertTrue(labels[2] >= 0);
    assertEquals(labels[2], labels[3]);
    assertNotEquals(labels[0], labels[2]);
    assertEquals(FuelDbscanClusterer.NOISE, labels[4]);
  }
}

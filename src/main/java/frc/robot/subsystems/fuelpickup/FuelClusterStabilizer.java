package frc.robot.subsystems.fuelpickup;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** Assigns persistent IDs to frame-local clusters using centroid association. */
public class FuelClusterStabilizer {
  private static class StableClusterState {
    Translation3d centroid;
    int missedFrames;

    StableClusterState(Translation3d centroid) {
      this.centroid = centroid;
      this.missedFrames = 0;
    }
  }

  private record Candidate(int rawId, int stableId, double distanceMeters) {}

  private final double maxAssociationDistanceMeters;
  private final int maxMissedFrames;
  private final double centroidEmaAlpha;
  private final Map<Integer, StableClusterState> stableClusters = new HashMap<>();
  private int nextStableId = 0;

  public FuelClusterStabilizer(
      double maxAssociationDistanceMeters, int maxMissedFrames, double centroidEmaAlpha) {
    this.maxAssociationDistanceMeters = maxAssociationDistanceMeters;
    this.maxMissedFrames = maxMissedFrames;
    this.centroidEmaAlpha = centroidEmaAlpha;
  }

  /**
   * Updates persistent cluster state and returns a mapping from raw cluster IDs to stable IDs.
   *
   * @param rawCentroids map of per-frame DBSCAN cluster ID -> centroid
   */
  public Map<Integer, Integer> update(Map<Integer, Translation3d> rawCentroids) {
    List<Candidate> candidates = new ArrayList<>();
    for (Map.Entry<Integer, Translation3d> rawEntry : rawCentroids.entrySet()) {
      for (Map.Entry<Integer, StableClusterState> stableEntry : stableClusters.entrySet()) {
        double distance = rawEntry.getValue().getDistance(stableEntry.getValue().centroid);
        if (distance <= maxAssociationDistanceMeters) {
          candidates.add(new Candidate(rawEntry.getKey(), stableEntry.getKey(), distance));
        }
      }
    }
    candidates.sort(Comparator.comparingDouble(Candidate::distanceMeters));

    Map<Integer, Integer> rawToStable = new HashMap<>();
    Set<Integer> usedRaw = new HashSet<>();
    Set<Integer> usedStable = new HashSet<>();
    for (Candidate candidate : candidates) {
      if (usedRaw.contains(candidate.rawId()) || usedStable.contains(candidate.stableId())) {
        continue;
      }
      usedRaw.add(candidate.rawId());
      usedStable.add(candidate.stableId());
      rawToStable.put(candidate.rawId(), candidate.stableId());
    }

    Set<Integer> touchedStableIds = new HashSet<>();
    for (Map.Entry<Integer, Translation3d> rawEntry : rawCentroids.entrySet()) {
      int rawId = rawEntry.getKey();
      Translation3d rawCentroid = rawEntry.getValue();
      int stableId =
          rawToStable.computeIfAbsent(
              rawId,
              ignored -> {
                int newId = nextStableId++;
                stableClusters.put(newId, new StableClusterState(rawCentroid));
                return newId;
              });

      StableClusterState state = stableClusters.get(stableId);
      state.centroid = interpolate(state.centroid, rawCentroid, centroidEmaAlpha);
      state.missedFrames = 0;
      touchedStableIds.add(stableId);
    }

    for (int stableId : new ArrayList<>(stableClusters.keySet())) {
      if (touchedStableIds.contains(stableId)) {
        continue;
      }
      StableClusterState state = stableClusters.get(stableId);
      state.missedFrames++;
      if (state.missedFrames > maxMissedFrames) {
        stableClusters.remove(stableId);
      }
    }

    return rawToStable;
  }

  private Translation3d interpolate(Translation3d a, Translation3d b, double t) {
    double clampedT = Math.max(0.0, Math.min(1.0, t));
    return a.times(1.0 - clampedT).plus(b.times(clampedT));
  }
}

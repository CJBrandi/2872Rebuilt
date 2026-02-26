package frc.robot.subsystems.fuelpickup;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Lightweight ByteTrack-style tracker for persistent fuel labels in simulation.
 *
 * <p>This implementation performs high-confidence matching first, then a low-confidence second pass
 * against unmatched confirmed tracks.
 */
public class FuelBallTracker {
  public record Detection(double xPx, double yPx, double radiusPx, double confidence) {}

  public record TrackEstimate(
      int id, double xPx, double yPx, double radiusPx, double confidence, boolean confirmed) {}

  public record TrackerOutput(List<TrackEstimate> allTracks, List<TrackEstimate> confirmedTracks) {}

  private static class TrackInternal {
    int id;
    double xPx;
    double yPx;
    double vxPxPerSec;
    double vyPxPerSec;
    double radiusPx;
    double confidence;
    int hits;
    int missed;

    TrackInternal(int id, Detection detection) {
      this.id = id;
      this.xPx = detection.xPx();
      this.yPx = detection.yPx();
      this.radiusPx = detection.radiusPx();
      this.confidence = detection.confidence();
      this.hits = 1;
      this.missed = 0;
    }

    void predict(double dtSecs) {
      xPx += vxPxPerSec * dtSecs;
      yPx += vyPxPerSec * dtSecs;
      vxPxPerSec *= 0.96;
      vyPxPerSec *= 0.96;
    }

    void update(Detection detection, double dtSecs) {
      double measuredVx = (detection.xPx() - xPx) / dtSecs;
      double measuredVy = (detection.yPx() - yPx) / dtSecs;
      vxPxPerSec = 0.7 * vxPxPerSec + 0.3 * measuredVx;
      vyPxPerSec = 0.7 * vyPxPerSec + 0.3 * measuredVy;
      xPx = detection.xPx();
      yPx = detection.yPx();
      radiusPx = 0.8 * radiusPx + 0.2 * detection.radiusPx();
      confidence = detection.confidence();
      hits++;
      missed = 0;
    }
  }

  private record MatchCandidate(int trackIndex, int detectionIndex, double distancePx) {}

  private final double highConfidenceThreshold;
  private final double lowConfidenceThreshold;
  private final double primaryMatchDistancePx;
  private final double secondaryMatchDistancePx;
  private final double spawnSuppressionDistancePx;
  private final int minConfirmHits;
  private final int maxMissedFrames;

  private final List<TrackInternal> tracks = new ArrayList<>();
  private int nextTrackId = 1;
  private double lastTimestampSecs = Double.NaN;

  public FuelBallTracker(
      double highConfidenceThreshold,
      double lowConfidenceThreshold,
      double primaryMatchDistancePx,
      double secondaryMatchDistancePx,
      int minConfirmHits,
      int maxMissedFrames) {
    this(
        highConfidenceThreshold,
        lowConfidenceThreshold,
        primaryMatchDistancePx,
        secondaryMatchDistancePx,
        0.0,
        minConfirmHits,
        maxMissedFrames);
  }

  public FuelBallTracker(
      double highConfidenceThreshold,
      double lowConfidenceThreshold,
      double primaryMatchDistancePx,
      double secondaryMatchDistancePx,
      double spawnSuppressionDistancePx,
      int minConfirmHits,
      int maxMissedFrames) {
    this.highConfidenceThreshold = highConfidenceThreshold;
    this.lowConfidenceThreshold = lowConfidenceThreshold;
    this.primaryMatchDistancePx = primaryMatchDistancePx;
    this.secondaryMatchDistancePx = secondaryMatchDistancePx;
    this.spawnSuppressionDistancePx = spawnSuppressionDistancePx;
    this.minConfirmHits = minConfirmHits;
    this.maxMissedFrames = maxMissedFrames;
  }

  public TrackerOutput update(List<Detection> detections, double timestampSecs) {
    double dtSecs =
        Double.isNaN(lastTimestampSecs) ? 0.02 : Math.max(1e-3, timestampSecs - lastTimestampSecs);
    lastTimestampSecs = timestampSecs;

    for (TrackInternal track : tracks) {
      track.predict(dtSecs);
    }

    List<Integer> highConfidenceDetections = new ArrayList<>();
    List<Integer> lowConfidenceDetections = new ArrayList<>();
    for (int i = 0; i < detections.size(); i++) {
      double confidence = detections.get(i).confidence();
      if (confidence >= highConfidenceThreshold) {
        highConfidenceDetections.add(i);
      } else if (confidence >= lowConfidenceThreshold) {
        lowConfidenceDetections.add(i);
      }
    }

    List<Integer> allTrackIndexes = new ArrayList<>();
    for (int i = 0; i < tracks.size(); i++) {
      allTrackIndexes.add(i);
    }

    MatchResult primaryMatches =
        associate(allTrackIndexes, highConfidenceDetections, detections, primaryMatchDistancePx);
    applyMatches(primaryMatches.matches(), detections, dtSecs);

    List<Integer> confirmedUnmatchedTracks = new ArrayList<>();
    for (int trackIndex : primaryMatches.unmatchedTrackIndexes()) {
      if (isConfirmed(tracks.get(trackIndex))) {
        confirmedUnmatchedTracks.add(trackIndex);
      }
    }

    MatchResult secondaryMatches =
        associate(
            confirmedUnmatchedTracks,
            lowConfidenceDetections,
            detections,
            secondaryMatchDistancePx);
    applyMatches(secondaryMatches.matches(), detections, dtSecs);

    Set<Integer> matchedTracks = new HashSet<>();
    for (MatchCandidate match : primaryMatches.matches()) {
      matchedTracks.add(match.trackIndex());
    }
    for (MatchCandidate match : secondaryMatches.matches()) {
      matchedTracks.add(match.trackIndex());
    }

    for (int i = tracks.size() - 1; i >= 0; i--) {
      TrackInternal track = tracks.get(i);
      if (!matchedTracks.contains(i)) {
        track.missed++;
      }
      if (track.missed > maxMissedFrames) {
        tracks.remove(i);
      }
    }

    Set<Integer> matchedDetections = new HashSet<>();
    for (MatchCandidate match : primaryMatches.matches()) {
      matchedDetections.add(match.detectionIndex());
    }
    for (MatchCandidate match : secondaryMatches.matches()) {
      matchedDetections.add(match.detectionIndex());
    }
    for (int detectionIndex : highConfidenceDetections) {
      if (!matchedDetections.contains(detectionIndex)
          && shouldSpawnTrack(detections.get(detectionIndex))) {
        tracks.add(new TrackInternal(nextTrackId++, detections.get(detectionIndex)));
      }
    }
    for (int detectionIndex : lowConfidenceDetections) {
      if (!matchedDetections.contains(detectionIndex)
          && shouldSpawnTrack(detections.get(detectionIndex))) {
        tracks.add(new TrackInternal(nextTrackId++, detections.get(detectionIndex)));
      }
    }

    List<TrackEstimate> allTrackEstimates = new ArrayList<>();
    List<TrackEstimate> confirmedTrackEstimates = new ArrayList<>();
    for (TrackInternal track : tracks) {
      TrackEstimate estimate =
          new TrackEstimate(
              track.id,
              track.xPx,
              track.yPx,
              Math.max(track.radiusPx, 1.0),
              track.confidence,
              isConfirmed(track));
      allTrackEstimates.add(estimate);
      if (estimate.confirmed()) {
        confirmedTrackEstimates.add(estimate);
      }
    }

    return new TrackerOutput(List.copyOf(allTrackEstimates), List.copyOf(confirmedTrackEstimates));
  }

  private void applyMatches(
      List<MatchCandidate> matches, List<Detection> detections, double dtSecs) {
    for (MatchCandidate match : matches) {
      tracks.get(match.trackIndex()).update(detections.get(match.detectionIndex()), dtSecs);
    }
  }

  private MatchResult associate(
      List<Integer> trackIndexes,
      List<Integer> detectionIndexes,
      List<Detection> detections,
      double maxDistancePx) {
    List<MatchCandidate> candidates = new ArrayList<>();
    for (int trackIndex : trackIndexes) {
      TrackInternal track = tracks.get(trackIndex);
      for (int detectionIndex : detectionIndexes) {
        Detection detection = detections.get(detectionIndex);
        double distancePx =
            new Translation2d(track.xPx, track.yPx)
                .getDistance(new Translation2d(detection.xPx(), detection.yPx()));
        if (distancePx <= maxDistancePx) {
          candidates.add(new MatchCandidate(trackIndex, detectionIndex, distancePx));
        }
      }
    }

    candidates.sort(Comparator.comparingDouble(MatchCandidate::distancePx));
    Set<Integer> usedTracks = new HashSet<>();
    Set<Integer> usedDetections = new HashSet<>();
    List<MatchCandidate> acceptedMatches = new ArrayList<>();
    for (MatchCandidate candidate : candidates) {
      if (usedTracks.contains(candidate.trackIndex())
          || usedDetections.contains(candidate.detectionIndex())) {
        continue;
      }
      usedTracks.add(candidate.trackIndex());
      usedDetections.add(candidate.detectionIndex());
      acceptedMatches.add(candidate);
    }

    List<Integer> unmatchedTracks = new ArrayList<>();
    for (int trackIndex : trackIndexes) {
      if (!usedTracks.contains(trackIndex)) {
        unmatchedTracks.add(trackIndex);
      }
    }

    return new MatchResult(List.copyOf(acceptedMatches), List.copyOf(unmatchedTracks));
  }

  private boolean isConfirmed(TrackInternal track) {
    return track.hits >= minConfirmHits;
  }

  private boolean shouldSpawnTrack(Detection detection) {
    if (spawnSuppressionDistancePx <= 0.0) {
      return true;
    }
    Translation2d detectionPoint = new Translation2d(detection.xPx(), detection.yPx());
    for (TrackInternal track : tracks) {
      if (detectionPoint.getDistance(new Translation2d(track.xPx, track.yPx))
          <= spawnSuppressionDistancePx) {
        return false;
      }
    }
    return true;
  }

  private record MatchResult(List<MatchCandidate> matches, List<Integer> unmatchedTrackIndexes) {}
}

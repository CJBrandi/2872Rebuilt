package frc.robot.subsystems.fuelpickup;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Optimized DBSCAN implementation using spatial hashing for neighborhood lookup. */
public class FuelDbscanClusterer {
  public static final int UNVISITED = -2;
  public static final int NOISE = -1;

  public record ClusterResult(int[] labels, int clusterCount) {}

  private final double epsilonMeters;
  private final double epsilonSquaredMeters;
  private final int minPoints;

  public FuelDbscanClusterer(double epsilonMeters, int minPoints) {
    this.epsilonMeters = epsilonMeters;
    this.epsilonSquaredMeters = epsilonMeters * epsilonMeters;
    this.minPoints = minPoints;
  }

  public ClusterResult cluster(List<Translation3d> points) {
    int pointCount = points.size();
    int[] labels = new int[pointCount];
    Arrays.fill(labels, UNVISITED);
    if (pointCount == 0) {
      return new ClusterResult(labels, 0);
    }

    Map<Long, List<Integer>> grid = buildSpatialHash(points);
    int clusterId = 0;
    for (int i = 0; i < pointCount; i++) {
      if (labels[i] != UNVISITED) {
        continue;
      }

      List<Integer> neighbors = queryNeighbors(i, points, grid);
      if (neighbors.size() < minPoints) {
        labels[i] = NOISE;
        continue;
      }

      expandCluster(i, neighbors, clusterId, labels, points, grid);
      clusterId++;
    }

    return new ClusterResult(labels, clusterId);
  }

  private void expandCluster(
      int seedIndex,
      List<Integer> seedNeighbors,
      int clusterId,
      int[] labels,
      List<Translation3d> points,
      Map<Long, List<Integer>> grid) {
    labels[seedIndex] = clusterId;
    ArrayDeque<Integer> queue = new ArrayDeque<>(seedNeighbors);
    while (!queue.isEmpty()) {
      int index = queue.removeFirst();

      if (labels[index] == NOISE) {
        labels[index] = clusterId;
      }
      if (labels[index] != UNVISITED) {
        continue;
      }
      labels[index] = clusterId;

      List<Integer> neighbors = queryNeighbors(index, points, grid);
      if (neighbors.size() >= minPoints) {
        queue.addAll(neighbors);
      }
    }
  }

  private Map<Long, List<Integer>> buildSpatialHash(List<Translation3d> points) {
    Map<Long, List<Integer>> grid = new HashMap<>();
    for (int i = 0; i < points.size(); i++) {
      Translation3d point = points.get(i);
      long key = cellKey(cellX(point.getX()), cellY(point.getY()));
      grid.computeIfAbsent(key, ignored -> new ArrayList<>()).add(i);
    }
    return grid;
  }

  private List<Integer> queryNeighbors(
      int pointIndex, List<Translation3d> points, Map<Long, List<Integer>> grid) {
    Translation3d point = points.get(pointIndex);
    int centerX = cellX(point.getX());
    int centerY = cellY(point.getY());

    List<Integer> neighbors = new ArrayList<>();
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        List<Integer> candidates = grid.get(cellKey(centerX + dx, centerY + dy));
        if (candidates == null) {
          continue;
        }
        for (int candidateIndex : candidates) {
          if (squaredDistanceXY(point, points.get(candidateIndex)) <= epsilonSquaredMeters) {
            neighbors.add(candidateIndex);
          }
        }
      }
    }
    return neighbors;
  }

  private int cellX(double x) {
    return (int) Math.floor(x / epsilonMeters);
  }

  private int cellY(double y) {
    return (int) Math.floor(y / epsilonMeters);
  }

  private long cellKey(int x, int y) {
    return (((long) x) << 32) ^ (y & 0xffffffffL);
  }

  private double squaredDistanceXY(Translation3d a, Translation3d b) {
    double dx = a.getX() - b.getX();
    double dy = a.getY() - b.getY();
    return dx * dx + dy * dy;
  }
}

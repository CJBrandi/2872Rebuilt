// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.TagIO.PoseObservationType;
import frc.robot.util.LoggedTunableNumber;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Tag extends SubsystemBase {
  private final TagConsumer consumer;
  private final TagIO[] io;
  private final TagIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final LoggedTunableNumber[] poseEstimationEnabledTunables;

  public Tag(TagConsumer consumer, TagIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new TagIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new TagIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Tag camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    this.poseEstimationEnabledTunables = new LoggedTunableNumber[io.length];
    for (int i = 0; i < io.length; i++) {
      poseEstimationEnabledTunables[i] =
          new LoggedTunableNumber(
              "Tag/Camera" + Integer.toString(i) + "/PoseEstimationEnabled", 1.0);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Tag/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    List<Pose3d> allCameraFieldPoses = new LinkedList<>();
    Pose3d robotFieldPose = new Pose3d(RobotState.getInstance().getRobotPose());

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
      boolean poseEstimationEnabled = poseEstimationEnabledTunables[cameraIndex].get() > 0.5;

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        double cameraStdDevFactor = getCameraStdDevFactor(cameraIndex);
        linearStdDev *= cameraStdDevFactor;
        angularStdDev *= cameraStdDevFactor;

        // Send vision observation
        if (poseEstimationEnabled) {
          consumer.accept(
              observation.pose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, Double.POSITIVE_INFINITY));
        }
      }

      // Log camera metadata
      Logger.recordOutput(
          "Tag/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Tag/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Tag/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Tag/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Tag/Camera" + Integer.toString(cameraIndex) + "/CameraRobotRelativePose",
          inputs[cameraIndex].cameraRobotRelativePose);
      Logger.recordOutput(
          "Tag/Camera" + Integer.toString(cameraIndex) + "/PoseEstimationEnabled",
          poseEstimationEnabled);

      Pose3d cameraFieldPose =
          robotFieldPose.plus(
              new Transform3d(
                  inputs[cameraIndex].cameraRobotRelativePose.getTranslation(),
                  inputs[cameraIndex].cameraRobotRelativePose.getRotation()));
      Logger.recordOutput(
          "Tag/Camera" + Integer.toString(cameraIndex) + "/CameraFieldPose", cameraFieldPose);

      Translation3d cameraForwardMeters =
          new Translation3d(0.4, 0.0, 0.0).rotateBy(cameraFieldPose.getRotation());
      Pose3d cameraForwardEndpoint =
          new Pose3d(
              cameraFieldPose.getTranslation().plus(cameraForwardMeters),
              cameraFieldPose.getRotation());
      Logger.recordOutput(
          "Tag/Camera" + Integer.toString(cameraIndex) + "/CameraForwardAxis",
          cameraFieldPose,
          cameraForwardEndpoint);

      if (inputs[cameraIndex].turretMounted) {
        Logger.recordOutput(
            "Tag/Camera" + Integer.toString(cameraIndex) + "/TurretAxisRobotRelativePose",
            inputs[cameraIndex].turretAxisRobotRelativePose);
        Logger.recordOutput(
            "Tag/Camera" + Integer.toString(cameraIndex) + "/TurretPositionRobotRelativePose",
            inputs[cameraIndex].turretRobotRelativePose);

        Pose3d turretAxisFieldPose =
            robotFieldPose.plus(
                new Transform3d(
                    inputs[cameraIndex].turretAxisRobotRelativePose.getTranslation(),
                    inputs[cameraIndex].turretAxisRobotRelativePose.getRotation()));
        Pose3d turretPositionFieldPose =
            robotFieldPose.plus(
                new Transform3d(
                    inputs[cameraIndex].turretRobotRelativePose.getTranslation(),
                    inputs[cameraIndex].turretRobotRelativePose.getRotation()));

        Logger.recordOutput(
            "Tag/Camera" + Integer.toString(cameraIndex) + "/TurretAxisFieldPose",
            turretAxisFieldPose);
        Logger.recordOutput(
            "Tag/Camera" + Integer.toString(cameraIndex) + "/TurretPositionFieldPose",
            turretPositionFieldPose);
        Logger.recordOutput(
            "Tag/Camera" + Integer.toString(cameraIndex) + "/TurretAxisToCamera",
            turretAxisFieldPose,
            cameraFieldPose);
      }

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
      allCameraFieldPoses.add(cameraFieldPose);
    }

    // Log summary data
    Logger.recordOutput("Tag/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Tag/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Tag/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Tag/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    Logger.recordOutput("Tag/Summary/CameraFieldPoses", allCameraFieldPoses.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface TagConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}

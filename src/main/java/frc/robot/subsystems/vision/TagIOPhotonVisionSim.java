// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class TagIOPhotonVisionSim extends TagIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;
  private final TagCameraConfig cameraConfig;

  /**
   * Creates a new TagIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public TagIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this(TagCameraConfig.fixed(name, robotToCamera), poseSupplier);
  }

  /**
   * Creates a new TagIOPhotonVisionSim.
   *
   * @param cameraConfig Per-camera configuration.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public TagIOPhotonVisionSim(TagCameraConfig cameraConfig, Supplier<Pose2d> poseSupplier) {
    super(cameraConfig);
    this.poseSupplier = poseSupplier;
    this.cameraConfig = cameraConfig;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    visionSim.addCamera(cameraSim, cameraConfig.robotToCamera(Timer.getFPGATimestamp()));
  }

  @Override
  public void updateInputs(TagIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    visionSim.adjustCamera(cameraSim, cameraConfig.robotToCamera(now));
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}

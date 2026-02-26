package frc.robot.subsystems.fuelpickup;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.FuelSim;
import org.junit.jupiter.api.Test;

class FuelCameraModelTest {
  @Test
  void projectionRoundTripsThroughGroundPlane() {
    FuelCameraModel cameraModel = new FuelCameraModel(640, 480, 63.3, 49.7);
    Pose3d cameraPose = new Pose3d(new Translation3d(2.0, 3.0, 0.57), new Rotation3d());
    Translation3d originalPoint = new Translation3d(4.0, 3.25, FuelSim.getFuelRadiusMeters());

    FuelCameraModel.PixelObservation projected =
        cameraModel.project(originalPoint, cameraPose, FuelSim.getFuelRadiusMeters()).orElseThrow();
    Translation3d estimatedPoint =
        cameraModel
            .backProjectToPlane(
                projected.xPx(), projected.yPx(), cameraPose, FuelSim.getFuelRadiusMeters())
            .orElseThrow();

    assertEquals(originalPoint.getX(), estimatedPoint.getX(), 0.05);
    assertEquals(originalPoint.getY(), estimatedPoint.getY(), 0.05);
    assertEquals(originalPoint.getZ(), estimatedPoint.getZ(), 1e-6);
  }

  @Test
  void rearFacingCameraSeesFuelBehindRobot() {
    FuelCameraModel cameraModel =
        new FuelCameraModel(
            FuelPickupConstants.frameWidthPx,
            FuelPickupConstants.frameHeightPx,
            FuelPickupConstants.horizontalFovDeg,
            FuelPickupConstants.verticalFovDeg);
    Pose3d cameraPose = new Pose3d().plus(FuelPickupConstants.robotToLimelight);
    Translation3d behindRobotFuel = new Translation3d(-1.2, 0.0, FuelSim.getFuelRadiusMeters());

    FuelCameraModel.PixelObservation projection =
        cameraModel
            .project(behindRobotFuel, cameraPose, FuelSim.getFuelRadiusMeters())
            .orElseThrow();

    assertEquals(FuelPickupConstants.frameWidthPx / 2.0, projection.xPx(), 40.0);
  }
}

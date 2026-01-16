package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import lombok.Getter;
import lombok.Setter;

public class RobotState {
  @Getter
  private static RobotState instance = new RobotState();

    @Getter @Setter private Pose2d pose = new Pose2d();
}

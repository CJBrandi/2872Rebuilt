// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import lombok.Getter;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final Mode simMode = Mode.REAL;
  @Getter public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = true;
  public static boolean disableHAL = false;

  // Robot dimensions for FuelSim (meters)
  public static class RobotDimensions {
    public static final double frameWidth = 0.6985; // 27.5 inches frame (without bumpers)
    public static final double frameLength = 0.6985; // 27.5 inches frame (without bumpers)
    public static final double bumperThickness = 0.089; // ~3.5 inches per side
    public static final double width = frameWidth + 2 * bumperThickness; // Total with bumpers
    public static final double length = frameLength + 2 * bumperThickness; // Total with bumpers
    public static final double bumperHeight = 0.2; // Height of bumpers
  }

  // Intake bounding box for FuelSim (robot-relative, meters)
  public static class IntakeBounds {
    public static final double xMin = 0.25; // Front of robot
    public static final double xMax = 0.40;
    public static final double yMin = -0.20;
    public static final double yMax = 0.20;
  }

  // Shooter launch height for FuelSim (meters)
  public static final double launchHeight = 0.5;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class IntakeConstants {
    public static final int canId = 30;
    public static final String canBus = "";
    public static final double stepUp = 5;
  }

  public static class ElevatorConstants {
    public static final double upPositionMeters = 0.31; // 0.31
    public static final double transitionPositionMeters = 0.270129;
    public static final double downPositionMeters = 0.0; // 0.0

    // Simulation constants
    public static final double G = 9.81;
    public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(90.0);
  }

  public static class SuperstructureConstants {
    public static class TurretConstants {
      public static final int canId = 60;
      public static final String canBus = "";
      public static final double reduction = 40.66;

      public static class HallEffectDegrees {
        public static final double leftHall = 0;
        public static final double middleHall = 135;
        public static final double rightHall = 270;
      }
    }

    public static class ShooterConstants {
      public static class FlywheelConstants {
        public static final int canId = 51;
        public static final String canBus = "";
        public static final double stepUp = 1.0;
      }

      public static class HoodConstants {
        public static final int canId = 52;
        public static final String canBus = "";
        public static final double reduction = 6.0;
      }
    }

    public static class IndexerConstants {
      public static final double reduction = 1.0;
    }
  }
}

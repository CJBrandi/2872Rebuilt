// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class TurretConstants {
    public static final int canId = 20;
    public static final String canBus = "";
    public static final double reduction = 5;
  }

  public static class ShooterConstants {
    // CAN IDs
    public static final int flywheelCanId = 21;
    public static final int hoodCanId = 22;
    public static final String hoodCanBus = "";

    // Flywheel: REV Vortex with 2:1 step-up (flywheel spins 2x motor speed)
    public static final double flywheelStepUp = 2.0;

    // Hood: Falcon 500 with reduction gearing
    public static final double hoodReduction = 50.0;
  }

  public static class IndexerConstants {
    public static final double reduction = 1.0;
  }
}

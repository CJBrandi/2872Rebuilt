package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class IntakeTest {
  private static final double EPSILON = 1e-9;

  @Test
  void trenchOverrideDeploysAndRestoresRequestedGoal() {
    Intake intake = new Intake(new PivotIO() {}, new RollerIO() {});

    intake.stow();
    assertEquals(Intake.stowedAngle.getRadians(), intake.getPivot().getGoal(), EPSILON);

    intake.setTrenchAutoDeployEnabled(true);
    assertEquals(Intake.groundAngle.getRadians(), intake.getPivot().getGoal(), EPSILON);

    intake.setTrenchAutoDeployEnabled(false);
    assertEquals(Intake.stowedAngle.getRadians(), intake.getPivot().getGoal(), EPSILON);
  }

  @Test
  void trenchOverrideIsSkippedWhenRequestedGoalIsAlreadyDeployed() {
    Intake intake = new Intake(new PivotIO() {}, new RollerIO() {});

    intake.deploy();

    assertFalse(intake.requiresTrenchAutoDeploy());
    assertEquals(Intake.groundAngle.getRadians(), intake.getPivot().getGoal(), EPSILON);

    intake.setTrenchAutoDeployEnabled(true);
    assertTrue(intake.isTrenchAutoDeployEnabled());
    assertEquals(Intake.groundAngle.getRadians(), intake.getPivot().getGoal(), EPSILON);
  }

  @Test
  void worstCaseDeployTimeTracksCurrentPivotProfile() {
    Intake intake = new Intake(new PivotIO() {}, new RollerIO() {});

    assertEquals(0.74, intake.getWorstCaseDeployTimeSecs(), 0.02);
  }
}

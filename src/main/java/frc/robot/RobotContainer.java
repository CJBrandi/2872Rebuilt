// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FullAutoFuelPickupCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.intake.PivotIO;
import frc.robot.subsystems.intake.PivotIOSim;
import frc.robot.subsystems.intake.Roller;
import frc.robot.subsystems.intake.RollerIO;
import frc.robot.subsystems.intake.RollerIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.indexer.IndexerIO;
import frc.robot.subsystems.superstructure.indexer.IndexerIOSim;
import frc.robot.subsystems.superstructure.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.superstructure.shooter.*;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.subsystems.superstructure.turret.TurretIO;
import frc.robot.subsystems.superstructure.turret.TurretIOSim;
import frc.robot.subsystems.superstructure.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Detection;
import frc.robot.subsystems.vision.DetectionIO;
import frc.robot.subsystems.vision.DetectionIOSim;
import frc.robot.subsystems.vision.Tag;
import frc.robot.subsystems.vision.TagIO;
import frc.robot.subsystems.vision.TagIOPhotonVisionSim;
import frc.robot.util.FuelSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Superstructure superstructure;
  private final Elevator elevator;
  private final Pivot pivot;
  private final Roller roller;
  private Tag tag;
  private Detection detection;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Shooting toggle state
  private boolean continuousShootingEnabled = false;
  private static final double SHOT_PERIOD_SECONDS = 1.0 / 15.0; // 15 balls/second
  private double lastShotTime = 0.0;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        /*
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        superstructure =
            new Superstructure(
                new Shooter(
                    new FlywheelIOTalonFX(
                        Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants.canId,
                        Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants
                            .followerCanId,
                        Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants
                            .canBus),
                    new HoodIOTalonFX(
                        Constants.SuperstructureConstants.ShooterConstants.HoodConstants.canId,
                        Constants.SuperstructureConstants.ShooterConstants.HoodConstants.canBus)),
                new Turret(
                    new TurretIOTalonFX(
                        Constants.SuperstructureConstants.TurretConstants.canId,
                        Constants.SuperstructureConstants.TurretConstants.canBus)),
                new Indexer(
                    new IndexerIOTalonFX(
                        Constants.SuperstructureConstants.IndexerConstants.canId,
                        Constants.SuperstructureConstants.IndexerConstants.canBus)));

        elevator =
            new Elevator(
                new ElevatorIOKraken(
                    Constants.ElevatorConstants.canId,
                    Constants.ElevatorConstants.followerCanId,
                    Constants.ElevatorConstants.canBus));

        pivot =
            new Pivot(
                new PivotIOTalonFX(
                    Constants.IntakeConstants.PivotConstants.canId,
                    Constants.IntakeConstants.canBus));
        roller =
            new Roller(
                new RollerIOTalonFX(
                    Constants.IntakeConstants.RollerConstants.canId,
                    Constants.IntakeConstants.RollerConstants.canRangeId,
                    Constants.IntakeConstants.canBus));

        tag =
            new Tag(
                drive::addVisionMeasurement, new TagIOPhotonVision(camera0Name, robotToCamera0));

        detection = new Detection(drive::getPose, new DetectionIOLimeLight("limelight", "fuel"));

         */

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        superstructure =
            new Superstructure(
                new Shooter(
                    new FlywheelIOTalonFX(
                        Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants.canId,
                        Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants
                            .followerCanId,
                        Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants
                            .canBus),
                    new HoodIOTalonFX(
                        Constants.SuperstructureConstants.ShooterConstants.HoodConstants.canId,
                        Constants.SuperstructureConstants.ShooterConstants.HoodConstants.canBus)),
                new Turret(
                    new TurretIOTalonFX(
                        Constants.SuperstructureConstants.TurretConstants.canId,
                        Constants.SuperstructureConstants.TurretConstants.canBus)),
                new Indexer(
                    new IndexerIOTalonFX(
                        Constants.SuperstructureConstants.IndexerConstants.canId,
                        Constants.SuperstructureConstants.IndexerConstants.canBus)));

        elevator = new Elevator(new ElevatorIOSim());

        pivot = new Pivot(new PivotIOSim());
        roller = new Roller(new RollerIOSim(DCMotor.getKrakenX44(1), 1.0, 0.001));

        tag =
            new Tag(
                drive::addVisionMeasurement,
                new TagIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new TagIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        detection = new Detection(drive::getPose, new DetectionIOSim(drive::getPose));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        superstructure =
            new Superstructure(
                new Shooter(new FlywheelIOSim(), new HoodIOSim()),
                new Turret(new TurretIOSim()),
                new Indexer(new IndexerIOSim()));

        elevator = new Elevator(new ElevatorIOSim());

        pivot = new Pivot(new PivotIOSim());
        roller = new Roller(new RollerIOSim(DCMotor.getKrakenX44(1), 1.0, 0.001));

        tag =
            new Tag(
                drive::addVisionMeasurement,
                new TagIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new TagIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        detection = new Detection(drive::getPose, new DetectionIOSim(drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        superstructure =
            new Superstructure(
                new Shooter(new FlywheelIO() {}, new HoodIO() {}),
                new Turret(new TurretIO() {}),
                new Indexer(new IndexerIO() {}));

        elevator = new Elevator(new ElevatorIO() {});

        pivot = new Pivot(new PivotIO() {});
        roller = new Roller(new RollerIO() {});

        tag = new Tag(drive::addVisionMeasurement, new TagIO() {}, new TagIO() {});
        detection = new Detection(drive::getPose, new DetectionIO() {});
        break;
    }

    // Initialize FuelSim in simulation mode
    if (Constants.currentMode == Constants.Mode.SIM) {
      FuelSim fuelSim = FuelSim.getInstance();

      // Register robot for collision detection
      fuelSim.registerRobot(
          Constants.RobotDimensions.width,
          Constants.RobotDimensions.length,
          Constants.RobotDimensions.bumperHeight,
          drive::getPose,
          () -> {
            ChassisSpeeds robotRelativeSpeeds = RobotState.getInstance().getRobotVelocity();
            Rotation2d robotHeading = drive.getPose().getRotation();
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond,
                robotRelativeSpeeds.omegaRadiansPerSecond,
                robotHeading);
          });

      // Register intake bounding box.
      // Camera measurement is the left corner of the intake in robot frame (+X forward, +Y left).
      // Intake spans full robot width from that left edge and extends rearward by a fixed depth.
      final double intakeLeftY = robotToDetectionCamera.getY();
      final double intakeRightY = intakeLeftY - Constants.RobotDimensions.width;
      final double intakeCameraEdgeX = robotToDetectionCamera.getX();
      final double robotRearEdgeX = -Constants.RobotDimensions.length / 2.0;
      final double intakeXMin = Math.min(intakeCameraEdgeX, robotRearEdgeX);
      final double intakeXMax = Math.max(intakeCameraEdgeX, robotRearEdgeX);
      final double intakeYMin = Math.min(intakeRightY, intakeLeftY);
      final double intakeYMax = Math.max(intakeRightY, intakeLeftY);

      fuelSim.registerIntake(
          intakeXMin,
          intakeXMax,
          intakeYMin,
          intakeYMax,
          () ->
              pivot.getAngle().getDegrees() <= Constants.IntakeBounds.maxDeployAngleDeg
                  && roller.getVelocityRadsPerSec()
                      > Constants.IntakeBounds.intakeActiveVelocityRadPerSec,
          superstructure::addFuelSimIntaked);

      // Enable air resistance for more realistic physics
      fuelSim.enableAirResistance();

      // Spawn starting fuel and start simulation
      fuelSim.spawnStartingFuel();
      fuelSim.start();
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Elevator Static Characterization", elevator.staticCharacterization(2));
    autoChooser.addOption("Elevator Homing", elevator.homingSequence());

    // Hood characterization/homing routines
    autoChooser.addOption("Hood Homing", superstructure.getShooter().hoodHomingCommand());
    autoChooser.addOption(
        "Hood Static Characterization",
        superstructure.getShooter().hoodStaticCharacterizationCommand(-2));

    autoChooser.addOption(
        "Turret Static Characterization", superstructure.getTurret().staticCharacterization(2.0));
    autoChooser.addOption(
        "Indexer Static Characterization", superstructure.getIndexer().staticCharacterization(2.0));

    // Hood characterization/homing routines
    autoChooser.addOption("Hood Homing", superstructure.getShooter().hoodHomingCommand());
    autoChooser.addOption(
        "Hood Static Characterization",
        superstructure.getShooter().hoodStaticCharacterizationCommand(-2));

    autoChooser.addOption(
        "Turret Static Characterization", superstructure.getTurret().staticCharacterization(2.0));

    // Intake characterization
    autoChooser.addOption(
        "Intake Pivot Static Characterization", pivot.staticCharacterization(2.0));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            this::getDriverOmegaInput));

    controller
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  continuousShootingEnabled = !continuousShootingEnabled;
                }));

    superstructure.setDefaultCommand(
        Commands.run(
            () -> {
              if (continuousShootingEnabled) {
                double currentTime = Timer.getFPGATimestamp();
                if (currentTime - lastShotTime >= SHOT_PERIOD_SECONDS) {
                  superstructure.launchFuelSim();
                  lastShotTime = currentTime;
                }
              }
            },
            superstructure));

    controller
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Elevator target control with POV
    controller.pov(0).onTrue(elevator.setTarget(Elevator.Target.UP));
    controller.pov(90).onTrue(elevator.setTarget(Elevator.Target.TRANSITION));
    controller.pov(180).onTrue(elevator.setTarget(Elevator.Target.DOWN));

    // Intake control
    controller.y().whileTrue(new FullAutoFuelPickupCommand(drive, pivot, roller));

    controller
        .b()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  pivot.setGoal(() -> Intake.groundAngle);
                  roller.runIntake();
                },
                () -> {
                  pivot.setGoal(() -> Intake.stowedAngle);
                  roller.stop();
                },
                pivot,
                roller));
    controller.rightBumper().whileTrue(Commands.runEnd(roller::runEject, roller::stop, roller));
  }

  private double getDriverOmegaInput() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return -controller.getHID().getRawAxis(Constants.DriverController.simOmegaAxis);
    }
    return -controller.getRightX();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command getHomingCommand() {
    return Commands.parallel(
        superstructure.getShooter().hoodHomingCommand(),
        superstructure.getTurret().homingSequence(),
        elevator.homingSequence(),
        pivot.homingSequence());
  }
}

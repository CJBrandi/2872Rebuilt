// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.hardware.traits.CommonDevice;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.*;
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
import frc.robot.subsystems.vision.*;
import frc.robot.util.Autos;
import frc.robot.util.FuelSim;
import java.util.ArrayList;
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
  private final Intake intake;
  private Tag tag;
  private Detection detection;
  private final ArrayList<CommonDevice> orchestraMotors = new ArrayList<>();
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private static final double SHOT_PERIOD_SECONDS = 1.0 / 9.0; // 15 balls/second
  private double lastShotTime = 0.0;

  private final LoggedDashboardChooser<Command> characterizer;
  private final LoggedDashboardChooser<Command> auto;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    TagCameraConfig runtimeCamera0Config =
        camera0Config.withTurretAngleAtTimestamp(
            timestampSeconds -> RobotState.getInstance().getTurretAngleAtTime(timestampSeconds));
    TagCameraConfig runtimeCamera1Config = camera1Config;
    TagCameraConfig runtimeCamera2Config = camera2config;

    switch (Constants.getCurrentMode()) {
      case REAL:
        ModuleIOTalonFX frontLeftModule = new ModuleIOTalonFX(TunerConstants.FrontLeft);
        ModuleIOTalonFX frontRightModule = new ModuleIOTalonFX(TunerConstants.FrontRight);
        ModuleIOTalonFX backLeftModule = new ModuleIOTalonFX(TunerConstants.BackLeft);
        ModuleIOTalonFX backRightModule = new ModuleIOTalonFX(TunerConstants.BackRight);

        orchestraMotors.add(frontLeftModule.getDriveTalon());
        orchestraMotors.add(frontLeftModule.getTurnTalon());
        orchestraMotors.add(frontRightModule.getDriveTalon());
        orchestraMotors.add(frontRightModule.getTurnTalon());
        orchestraMotors.add(backLeftModule.getDriveTalon());
        orchestraMotors.add(backLeftModule.getTurnTalon());
        orchestraMotors.add(backRightModule.getDriveTalon());
        orchestraMotors.add(backRightModule.getTurnTalon());

        drive =
            new Drive(
                new GyroIOPigeon2(),
                frontLeftModule,
                frontRightModule,
                backLeftModule,
                backRightModule);

        PivotIOTalonFX pivotIO =
            new PivotIOTalonFX(
                Constants.IntakeConstants.PivotConstants.canId, Constants.IntakeConstants.canBus);
        RollerIOTalonFX rollerIO =
            new RollerIOTalonFX(
                Constants.IntakeConstants.RollerConstants.canId, Constants.IntakeConstants.canBus);
        orchestraMotors.add(pivotIO.getTalon());
        orchestraMotors.add(rollerIO.getTalon());
        intake = new Intake(pivotIO, rollerIO);

        FlywheelIOTalonFX flywheelIO =
            new FlywheelIOTalonFX(
                Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants.canId,
                Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants.followerCanId,
                Constants.SuperstructureConstants.ShooterConstants.FlywheelConstants.canBus);
        HoodIOTalonFX hoodIO =
            new HoodIOTalonFX(
                Constants.SuperstructureConstants.ShooterConstants.HoodConstants.canId,
                Constants.SuperstructureConstants.ShooterConstants.HoodConstants.canBus);
        TurretIOTalonFX turretIO =
            new TurretIOTalonFX(
                Constants.SuperstructureConstants.TurretConstants.canId,
                Constants.SuperstructureConstants.TurretConstants.canBus);
        IndexerIOTalonFX indexerIO =
            new IndexerIOTalonFX(
                Constants.SuperstructureConstants.IndexerConstants.canId,
                Constants.SuperstructureConstants.IndexerConstants.canBus);

        orchestraMotors.add(flywheelIO.getTalon());
        orchestraMotors.add(flywheelIO.getFollowerTalon());
        orchestraMotors.add(hoodIO.getTalon());
        orchestraMotors.add(turretIO.getTalon());
        orchestraMotors.add(indexerIO.getTalon());

        superstructure =
            new Superstructure(
                new Shooter(flywheelIO, hoodIO),
                new Turret(turretIO),
                new Indexer(indexerIO),
                intake);

        elevator = new Elevator(new ElevatorIO() {});
        tag =
            new Tag(
                drive::addVisionMeasurement,
                new TagIOPhotonVision(runtimeCamera0Config),
                new TagIOPhotonVision(runtimeCamera1Config),
                new TagIOPhotonVision(runtimeCamera2Config));
        /*
        detection =
            new Detection(
                drive::getPose, new DetectionIOLimeLight("limelight", "fuel", drive::getPose));


         */
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

        intake = new Intake(new PivotIOSim(), new RollerIOSim(DCMotor.getKrakenX44(1), 1.0, 0.001));

        superstructure =
            new Superstructure(
                new Shooter(new FlywheelIOSim(), new HoodIOSim()),
                new Turret(new TurretIOSim()),
                new Indexer(new IndexerIOSim()),
                intake);

        elevator = new Elevator(new ElevatorIOSim());

        tag =
            new Tag(
                drive::addVisionMeasurement,
                new TagIOPhotonVisionSim(runtimeCamera0Config, drive::getPose));
        // new TagIOPhotonVisionSim(runtimeCamera1Config, drive::getPose);
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

        intake = new Intake(new PivotIO() {}, new RollerIO() {});

        superstructure =
            new Superstructure(
                new Shooter(new FlywheelIO() {}, new HoodIO() {}),
                new Turret(new TurretIO() {}),
                new Indexer(new IndexerIO() {}),
                intake);

        elevator = new Elevator(new ElevatorIO() {});

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
              intake.getPivot().getAngle().getDegrees() <= Constants.IntakeBounds.maxDeployAngleDeg
                  && intake.getRoller().getVelocityRadsPerSec()
                      > Constants.IntakeBounds.intakeActiveVelocityRadPerSec,
          superstructure::addFuelSimIntaked);

      // Enable air resistance for more realistic physics
      fuelSim.enableAirResistance();

      // Spawn starting fuel and start simulation
      fuelSim.spawnStartingFuel();
      fuelSim.start();
      superstructure.setDefaultCommand(
          Commands.run(
              () -> {
                if (RobotState.getInstance().isAutoEmpty()) {
                  double currentTime = Timer.getFPGATimestamp();
                  if (currentTime - lastShotTime >= SHOT_PERIOD_SECONDS) {
                    superstructure.launchFuelSim();
                    lastShotTime = currentTime;
                  }
                }
              },
              superstructure));

      controller
          .button(1)
          .onTrue(
              Commands.runOnce(
                  () ->
                      RobotState.getInstance()
                          .setAutoEmpty(!RobotState.getInstance().isAutoEmpty())));
    }

    Autos autos = new Autos(drive, intake, superstructure);

    // Set up auto routines
    characterizer = new LoggedDashboardChooser<>("Characterization", new SendableChooser<>());
    auto = new LoggedDashboardChooser<>("Auto", new SendableChooser<>());

    auto.addOption("Right middle double", autos.RIGHT_MID_DOUBLE());
    auto.addOption("Right middle outpost", autos.RIGHT_MID_OUTPOST());
    auto.addOption("Left middle double", autos.LEFT_MID_DOUBLE());
    auto.addOption("Left middle depot", autos.LEFT_MID_DEPOT());
    // auto.addOption("Middle outpost", autos.MIDDLE_OUTPOST());
    // Set up SysId routines
    characterizer.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    characterizer.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    characterizer.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    characterizer.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    characterizer.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    characterizer.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    characterizer.addOption(
        "Elevator Static Characterization Up", elevator.staticCharacterization(2));
    characterizer.addOption(
        "Elevator Static Characterization Down", elevator.staticCharacterization(-2));
    characterizer.addOption("Elevator Homing", elevator.homingSequence());

    // Hood characterization/homing routines
    characterizer.addOption("Hood Homing", superstructure.getShooter().hoodHomingCommand());
    characterizer.addOption(
        "Hood Static Characterization",
        superstructure.getShooter().hoodStaticCharacterizationCommand(-2));

    characterizer.addOption(
        "Turret Static Characterization", superstructure.getTurret().staticCharacterization(2.0));
    characterizer.addOption(
        "Indexer Static Characterization", superstructure.getIndexer().staticCharacterization(2.0));

    // Intake characterization
    characterizer.addOption(
        "Intake Pivot Static Characterization", intake.staticCharacterization(2.0));

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
        DriveCommands.joystickDriveWithSnakeMode(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            controller.a()));

    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.leftBumper().onTrue(Commands.runOnce(intake::stow, intake));

    controller
        .leftTrigger()
        .onTrue(Commands.runOnce(intake::deploy, intake))
        .onFalse(Commands.runOnce(intake::toggleIntake, intake));

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> intake.getRoller().setTemporaryVelocityOverride(Roller.ejectVelocity.get()),
                intake.getRoller()))
        .onFalse(
            Commands.runOnce(
                intake.getRoller()::clearTemporaryVelocityOverride, intake.getRoller()));

    controller.x().onTrue(superstructure.getShooter().hoodHomingCommand());

    controller
        .rightTrigger()
        .onTrue(
            Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(true))
                .ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(false)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto.get();
  }

  public ArrayList<CommonDevice> getOrchestraMotors() {
    return orchestraMotors;
  }

  public Command getHomingCommand() {
    return Commands.parallel(
        superstructure.getShooter().hoodHomingCommand(), elevator.homingSequence());
    // intake.homingSequence());
  }
}

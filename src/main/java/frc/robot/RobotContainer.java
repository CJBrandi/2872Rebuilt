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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.indexer.IndexerIOSim;
import frc.robot.subsystems.superstructure.shooter.*;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.subsystems.superstructure.turret.TurretIO;
import frc.robot.subsystems.superstructure.turret.TurretIOSim;
import frc.robot.subsystems.superstructure.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  private Vision vision;
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
                            .canBus),
                    new HoodIOTalonFX(
                        Constants.SuperstructureConstants.ShooterConstants.HoodConstants.canId,
                        Constants.SuperstructureConstants.ShooterConstants.HoodConstants.canBus)),
                new Turret(
                    new TurretIOTalonFX(
                        Constants.SuperstructureConstants.TurretConstants.canId,
                        Constants.SuperstructureConstants.TurretConstants.canBus)),
                new Indexer(new IndexerIOSim()));

        elevator = new Elevator(new ElevatorIOKraken());

        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOPhotonVision(camera0Name, robotToCamera0));

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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
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
                new Indexer(new IndexerIOSim()));

        elevator = new Elevator(new ElevatorIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
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
          () -> RobotState.getInstance().getRobotVelocity());

      // Register intake bounding box
      fuelSim.registerIntake(
          Constants.IntakeBounds.xMin,
          Constants.IntakeBounds.xMax,
          Constants.IntakeBounds.yMin,
          Constants.IntakeBounds.yMax,
          () -> true); // TODO: wire to actual intake running state

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

    // Hood characterization/homing routines
    autoChooser.addOption("Hood Homing", superstructure.getShooter().hoodHomingCommand());
    autoChooser.addOption(
        "Hood Static Characterization",
        superstructure.getShooter().hoodStaticCharacterizationCommand(-2));

    autoChooser.addOption(
        "Turret Static Characterization", superstructure.getTurret().staticCharacterization(2.0));

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
        Commands.either(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()),
            Commands.none(),
            this::isHoodHomed));

    controller
        .x()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      continuousShootingEnabled = !continuousShootingEnabled;
                    })
                .onlyIf(this::isHoodHomed));

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
                superstructure)
            .onlyIf(this::isHoodHomed));

    controller
        .b()
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
    return superstructure.getShooter().hoodHomingCommand();
  }

  public boolean isHoodHomed() {
    return superstructure.getShooter().isHoodHomed();
  }
}

# FRC Documentation Index

This is a compact index for online lookup. Do not treat it as a copy of the docs; use it to find official pages and Javadocs quickly.

## WPILib

- Prose docs: https://docs.wpilib.org/
- Java API root: https://github.wpilib.org/allwpilib/docs/release/java/index.html
- All classes: https://github.wpilib.org/allwpilib/docs/release/java/allclasses-index.html
- Common packages:
  - `edu.wpi.first.wpilibj`
  - `edu.wpi.first.wpilibj2.command`
  - `edu.wpi.first.math`
  - `edu.wpi.first.math.geometry`
  - `edu.wpi.first.math.kinematics`
  - `edu.wpi.first.math.estimator`
  - `edu.wpi.first.networktables`
  - `edu.wpi.first.wpilibj.simulation`
- Common classes:
  - `edu.wpi.first.networktables.NetworkTableInstance`
  - `edu.wpi.first.wpilibj2.command.CommandScheduler`
  - `edu.wpi.first.math.estimator.SwerveDrivePoseEstimator`
  - `edu.wpi.first.wpilibj.simulation.DriverStationSim`
- Search templates:
  - `site:github.wpilib.org/allwpilib/docs/release/java CommandScheduler`
  - `site:github.wpilib.org/allwpilib/docs/release/java SwerveDrivePoseEstimator`
  - `site:github.wpilib.org/allwpilib/docs/release/java NetworkTableInstance`
  - `site:docs.wpilib.org command-based subsystem Java`
  - `site:docs.wpilib.org robot simulation Java DriverStationSim`

## REV

- REV docs: https://docs.revrobotics.com/
- REVLib Java API root: https://codedocs.revrobotics.com/java/index.html
- SPARK package: https://codedocs.revrobotics.com/java/com/revrobotics/spark/package-summary
- Common packages/classes:
  - `com.revrobotics.spark.SparkMax`
  - `com.revrobotics.spark.SparkFlex`
  - `com.revrobotics.spark.SparkBase`
  - `com.revrobotics.spark.SparkClosedLoopController`
  - `com.revrobotics.spark.config.SparkMaxConfig`
  - `com.revrobotics.spark.config.SparkFlexConfig`
  - `com.revrobotics.spark.SparkSim`
- Vendordep hints:
  - Names often include `REVLib`, `revrobotics`, or `REVRobotics`.
  - 2025+ Java code commonly uses `SparkMax`, `SparkFlex`, and config objects under `com.revrobotics.spark.config`.
- Search templates:
  - `site:codedocs.revrobotics.com/java SparkMax configure`
  - `site:codedocs.revrobotics.com/java SparkMaxConfig`
  - `site:docs.revrobotics.com REVLib SPARK Java closed loop`

## CTRE Phoenix 6

- Phoenix 6 docs: https://v6.docs.ctr-electronics.com/
- Latest Java API root: https://api.ctr-electronics.com/phoenix6/latest/java/
- Stable Java API root: https://api.ctr-electronics.com/phoenix6/stable/java/
- Release Java API root: https://api.ctr-electronics.com/phoenix6/release/java/
- Common packages/classes:
  - `com.ctre.phoenix6.hardware.TalonFX`
  - `com.ctre.phoenix6.hardware.TalonFXS`
  - `com.ctre.phoenix6.hardware.CANcoder`
  - `com.ctre.phoenix6.hardware.Pigeon2`
  - `com.ctre.phoenix6.CANBus`
  - `com.ctre.phoenix6.configs.*`
  - `com.ctre.phoenix6.controls.*`
  - `com.ctre.phoenix6.signals.*`
- Vendordep hints:
  - Names often include `Phoenix6`, `CTRE-Phoenix`, or `ctre`.
  - 2026+ code may prefer `CANBus` objects over string CAN bus constructors.
- Search templates:
  - `site:api.ctr-electronics.com/phoenix6/latest/java TalonFX constructor CANBus`
  - `site:api.ctr-electronics.com/phoenix6/latest/java MotionMagicVoltage`
  - `site:v6.docs.ctr-electronics.com Phoenix 6 Java control requests`

## Limelight

- Limelight docs: https://docs.limelightvision.io/
- Limelight Lib docs: https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib
- Limelight complete NetworkTables API: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
- Limelight Java library source: https://github.com/LimelightVision/limelightlib-wpijava
- Common Java entry points:
  - `LimelightHelpers`
  - `LimelightHelpers.getTX(name)`
  - `LimelightHelpers.getTY(name)`
  - `LimelightHelpers.getTA(name)`
  - `LimelightHelpers.getTV(name)`
  - `LimelightHelpers.getBotPose(name)`
  - `LimelightHelpers.getBotPose_wpiBlue(name)`
  - `LimelightHelpers.getBotPose_wpiRed(name)`
  - `LimelightHelpers.SetRobotOrientation(...)`
- Common NetworkTables table/keys:
  - table: `limelight` or a named camera table such as `limelight-front`
  - target validity: `tv`
  - offsets: `tx`, `ty`
  - target area: `ta`
  - latency: `tl`, `cl`
  - pose arrays: `botpose`, `botpose_wpiblue`, `botpose_wpired`, `botpose_orb_wpiblue`, `botpose_orb_wpired`
  - pipeline: `pipeline`
- Search templates:
  - `site:docs.limelightvision.io LimelightHelpers Java getBotPose`
  - `site:github.com/LimelightVision/limelightlib-wpijava LimelightHelpers`
  - `site:docs.limelightvision.io NetworkTables tx ty tv botpose`

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private MecanumDrive mecanumDrive;

  private SparkMax rearLeftMotor;
  private SparkMax frontLeftMotor;
  private SparkMax rearRightMotor;
  private SparkMax frontRightMotor;

  private AHRS navx;

  private MecanumDriveKinematics kinematics;

  private MecanumDriveOdometry odometry;

  private Field2d field;

 
  public DriveSubsystem(int frontLeftMotorID, int frontRightMotorID, int rearLeftMotorID, int rearRightMotorID, Pose2d initialPose) {
    rearLeftMotor = new SparkMax(rearLeftMotorID, MotorType.kBrushless);
    frontLeftMotor = new SparkMax(frontLeftMotorID, MotorType.kBrushless);
    rearRightMotor = new SparkMax(rearRightMotorID, MotorType.kBrushless);
    frontRightMotor = new SparkMax(frontRightMotorID, MotorType.kBrushless);
    
    SparkMaxConfig reversedConfig = new SparkMaxConfig();
    reversedConfig.inverted(true);
    SparkMaxConfig nonReversedConfig = new SparkMaxConfig();
    nonReversedConfig.inverted(false);
    
    rearLeftMotor.configure(MotorConstants.REAR_LEFT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontLeftMotor.configure(MotorConstants.FRONT_LEFT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rearRightMotor.configure(MotorConstants.REAR_RIGHT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(MotorConstants.FRONT_RIGHT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    navx = new AHRS(NavXComType.kMXP_SPI);
    
    kinematics = new MecanumDriveKinematics(
      DriveConstants.WHEEL_POSITIONS[0],
      DriveConstants.WHEEL_POSITIONS[1],
      DriveConstants.WHEEL_POSITIONS[2],
      DriveConstants.WHEEL_POSITIONS[3]
    );

    odometry = new MecanumDriveOdometry(kinematics, getHeading(), getWheelPositions(), initialPose);

    field = new Field2d();

    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
              new PIDConstants(5.0, 0, 0, 0),
              new PIDConstants(5.0, 0, 0, 0)
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this
    );

    SmartDashboard.putData(field);
    
  }

  public void drive(double ySpeed, double xSpeed, double zRotation) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  public void drive(double ySpeed, double xSpeed, double zRotation, Rotation2d gyroAngle) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, gyroAngle);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    setSpeeds(wheelSpeeds);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {    
    double frontLeftOutput = wheelSpeeds.frontLeftMetersPerSecond / DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double frontRightOutput = wheelSpeeds.frontRightMetersPerSecond / DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rearLeftOutput = wheelSpeeds.rearLeftMetersPerSecond / DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rearRightOutput = wheelSpeeds.rearRightMetersPerSecond / DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    
    frontLeftMotor.set(frontLeftOutput);
    frontRightMotor.set(frontRightOutput);
    rearLeftMotor.set(rearLeftOutput);
    rearRightMotor.set(rearRightOutput);
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getWheelPositions());
    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putNumber("Robot X", getPose().getX());
    SmartDashboard.putNumber("Robot Y", getPose().getY());
    SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(navx.getYaw()); //FIXME: Check if this needs to be negated
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
      getDistance(frontLeftMotor.getEncoder()),
      getDistance(frontRightMotor.getEncoder()),
      getDistance(rearLeftMotor.getEncoder()),
      getDistance(rearRightMotor.getEncoder())
    );
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      getVelocity(frontLeftMotor.getEncoder()),
      getVelocity(frontRightMotor.getEncoder()),
      getVelocity(rearLeftMotor.getEncoder()),
      getVelocity(rearRightMotor.getEncoder())
    );
  }

  public double getDistance(RelativeEncoder encoder) {
    return encoder.getPosition() / DriveConstants.GEARBOX_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE;
  }

  public double getVelocity(RelativeEncoder encoder) {
    return encoder.getVelocity() / DriveConstants.GEARBOX_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE / 60.0;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(getHeading(), getWheelPositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void resetEncoders() {
    frontLeftMotor.getEncoder().setPosition(0);
    frontRightMotor.getEncoder().setPosition(0);
    rearLeftMotor.getEncoder().setPosition(0);
    rearRightMotor.getEncoder().setPosition(0);
  }

  public void zeroHeading() {
    navx.reset();
  }

}

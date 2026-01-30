// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Commands.DriveCommand;
import frc.robot.Constants.*;

public class RobotContainer {
  private DriveSubsystem driveSubsystem = new DriveSubsystem(
      MotorConstants.REAR_LEFT_MOTOR_ID,
      MotorConstants.FRONT_LEFT_MOTOR_ID,
      MotorConstants.REAR_RIGHT_MOTOR_ID,
      MotorConstants.FRONT_RIGHT_MOTOR_ID,
      new Pose2d()
  );

  private Joystick driverJoystick = new Joystick(0);

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            () -> driverJoystick.getY(),
            () -> driverJoystick.getX(),
            () -> driverJoystick.getZ(),
            driveSubsystem
        )
    );
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  private void configureBindings() {}

  public void resetSensors() {
    driveSubsystem.zeroHeading();
    driveSubsystem.resetEncoders();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}

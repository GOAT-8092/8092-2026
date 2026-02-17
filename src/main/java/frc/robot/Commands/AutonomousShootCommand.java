package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class AutonomousShootCommand extends SequentialCommandGroup {
    public AutonomousShootCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,
                                  TurretSubsystem turretSubsystem, VisionSubsystem visionSubsystem) {
        addCommands(
            // Align turret to target
            new AlignToTargetCommand(driveSubsystem, turretSubsystem, visionSubsystem),
            // Shoot
            new ShootCommand(shooterSubsystem, 3.0)
        );
    }
}
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class AlignToTargetCommand extends Command {
    private DriveSubsystem driveSubsystem;
    private TurretSubsystem turretSubsystem;
    private VisionSubsystem visionSubsystem;
    private double tolerance = 2.0; // degrees

    public AlignToTargetCommand(DriveSubsystem driveSubsystem, TurretSubsystem turretSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, turretSubsystem);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {
            double horizontalOffset = visionSubsystem.getHorizontalOffset();
            double verticalOffset = visionSubsystem.getVerticalOffset();

            // Rotate turret
            double turretSpeed = horizontalOffset * 0.02; // Tune
            turretSubsystem.rotate(turretSpeed);

            // Optionally rotate robot if needed
            // double driveRotation = horizontalOffset * 0.01;
            // driveSubsystem.drive(0, 0, driveRotation);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stop();
        driveSubsystem.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return visionSubsystem.hasTarget() && Math.abs(visionSubsystem.getHorizontalOffset()) < tolerance;
    }
}
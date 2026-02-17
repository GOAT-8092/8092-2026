package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class TurretTrackCommand extends Command {
    private TurretSubsystem turretSubsystem;
    private VisionSubsystem visionSubsystem;

    public TurretTrackCommand(TurretSubsystem turretSubsystem, VisionSubsystem visionSubsystem) {
        this.turretSubsystem = turretSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {
            double horizontalOffset = visionSubsystem.getHorizontalOffset();
            double speed = horizontalOffset * 0.01; // Proportional control, tune this
            turretSubsystem.rotate(speed);
        } else {
            turretSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Continuous tracking
    }
}
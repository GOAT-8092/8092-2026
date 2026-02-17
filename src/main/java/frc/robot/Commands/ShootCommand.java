package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private double duration; // in seconds

    public ShootCommand(ShooterSubsystem shooterSubsystem, double duration) {
        this.shooterSubsystem = shooterSubsystem;
        this.duration = duration;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.shoot();
    }

    @Override
    public void execute() {
        // Keep shooting
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Or implement timing if needed
    }
}
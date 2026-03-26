package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, double duration) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    public ShootCommand(ShooterSubsystem shooterSubsystem, double duration) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = null;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.shoot();
        if (intakeSubsystem != null) {
            intakeSubsystem.depodanAticiyaYukariTasimaBaslat();
        }
    }

    @Override
    public void execute() {
        // Keep shooting
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
        if (intakeSubsystem != null) {
            intakeSubsystem.depodanAticiyaYukariTasimaDurdur();
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Or implement timing if needed
    }
}

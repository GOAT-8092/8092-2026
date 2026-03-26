package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import frc.robot.Sabitler.ModulSabitleri;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

class IntakeAndShootCommandTest {
    @Test
    @Tag("fast")
    void intakeCommandSetsAndStopsMotorOutput() {
        IntakeSubsystem subsystem = new IntakeSubsystem();
        IntakeCommand command = new IntakeCommand(subsystem);

        command.execute();
        assertEquals(ModulSabitleri.ALIM_HIZI, subsystem.getLastCommandedSpeed(), 1e-9);

        command.end(false);
        assertEquals(0.0, subsystem.getLastCommandedSpeed(), 1e-9);
    }

    @Test
    @Tag("fast")
    void shootCommandInitializesAndStopsShooterOutput() {
        ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        ShootCommand command = new ShootCommand(shooterSubsystem, intakeSubsystem, 3.0);

        command.initialize();
        assertEquals(ModulSabitleri.ATICI_HIZI, shooterSubsystem.getLastCommandedSpeed(), 1e-9);
        assertEquals(ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI, intakeSubsystem.getSonTasiyiciHizi(), 1e-9);

        command.end(false);
        assertEquals(0.0, shooterSubsystem.getLastCommandedSpeed(), 1e-9);
        assertEquals(0.0, intakeSubsystem.getSonTasiyiciHizi(), 1e-9);
    }
}

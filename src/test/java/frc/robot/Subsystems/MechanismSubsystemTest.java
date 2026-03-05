package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import frc.robot.Constants.ModuleConstants;

class MechanismSubsystemTest {
    @Test
    @Tag("fast")
    void intakeTracksLastCommand() {
        IntakeSubsystem intake = new IntakeSubsystem();
        intake.intake();
        assertEquals(ModuleConstants.INTAKE_SPEED, intake.getLastCommandedSpeed(), 1e-9);

        intake.outtake();
        assertEquals(-ModuleConstants.INTAKE_SPEED, intake.getLastCommandedSpeed(), 1e-9);
    }

    @Test
    @Tag("fast")
    void shooterTracksLastCommand() {
        ShooterSubsystem shooter = new ShooterSubsystem();
        shooter.shoot();
        assertEquals(ModuleConstants.SHOOTER_SPEED, shooter.getLastCommandedSpeed(), 1e-9);

        shooter.stop();
        assertEquals(0.0, shooter.getLastCommandedSpeed(), 1e-9);
    }

    @Test
    @Tag("fast")
    void turretSetAngleIssuesProportionalCommand() {
        TurretSubsystem turret = new TurretSubsystem();
        turret.setAngle(100.0);
        assertEquals(1.0, turret.getLastCommandedSpeed(), 1e-9);
        turret.stop();
        assertEquals(0.0, turret.getLastCommandedSpeed(), 1e-9);
    }
}

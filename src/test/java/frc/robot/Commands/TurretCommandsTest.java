package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

class TurretCommandsTest {
    @Test
    @Tag("fast")
    void turretTrackRotatesWhenTargetVisible() {
        TurretSubsystem turret = new TurretSubsystem();
        VisionSubsystem vision = mock(VisionSubsystem.class);
        when(vision.hasTarget()).thenReturn(true);
        when(vision.getHorizontalOffset()).thenReturn(10.0);

        TurretTrackCommand command = new TurretTrackCommand(turret, vision);
        command.execute();

        assertEquals(0.1, turret.getLastCommandedSpeed(), 1e-9);
    }

    @Test
    @Tag("fast")
    void alignToTargetStopsWhenEnded() {
        TurretSubsystem turret = new TurretSubsystem();
        VisionSubsystem vision = mock(VisionSubsystem.class);
        DriveSubsystem drive = mock(DriveSubsystem.class);

        AlignToTargetCommand command = new AlignToTargetCommand(drive, turret, vision);
        command.end(false);

        assertEquals(0.0, turret.getLastCommandedSpeed(), 1e-9);
    }
}

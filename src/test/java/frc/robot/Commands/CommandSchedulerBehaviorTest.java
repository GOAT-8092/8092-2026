package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class CommandSchedulerBehaviorTest {
    private static class TestSubsystem extends SubsystemBase {}

    private static class EndlessCommand extends Command {
        EndlessCommand(TestSubsystem subsystem) {
            addRequirements(subsystem);
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }

    @Test
    @Tag("fast")
    void secondCommandInterruptsFirstOnSameRequirement() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();

        TestSubsystem subsystem = new TestSubsystem();
        Command first = new EndlessCommand(subsystem);
        Command second = new EndlessCommand(subsystem);

        scheduler.schedule(first);
        scheduler.run();
        assertTrue(first.isScheduled());

        scheduler.schedule(second);
        scheduler.run();

        assertFalse(first.isScheduled());
        assertTrue(second.isScheduled());

        scheduler.cancelAll();
    }
}

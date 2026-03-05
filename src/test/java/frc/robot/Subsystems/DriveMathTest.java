package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import frc.robot.Constants.DriveConstants;

class DriveMathTest {
    @Test
    @Tag("fast")
    void encoderConversionsMatchExpectedUnits() {
        double meters = DriveMath.encoderPositionToMeters(127.5, DriveConstants.GEARBOX_RATIO, DriveConstants.WHEEL_CIRCUMFERENCE);
        assertEquals(10.0 * DriveConstants.WHEEL_CIRCUMFERENCE, meters, 1e-9);

        double mps = DriveMath.encoderVelocityRpmToMetersPerSecond(127.5, DriveConstants.GEARBOX_RATIO, DriveConstants.WHEEL_CIRCUMFERENCE);
        assertEquals((10.0 * DriveConstants.WHEEL_CIRCUMFERENCE) / 60.0, mps, 1e-9);
    }

    @Test
    @Tag("fast")
    void wrapDeltaDegreesHandlesCrossingBoundary() {
        assertEquals(20.0, DriveSubsystem.wrapDeltaDegrees(170.0, -170.0), 1e-9);
        assertEquals(-20.0, DriveSubsystem.wrapDeltaDegrees(-170.0, 170.0), 1e-9);
    }
}

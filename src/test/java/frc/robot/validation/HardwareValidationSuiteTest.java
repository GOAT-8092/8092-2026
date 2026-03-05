package frc.robot.validation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.time.Clock;
import java.time.Instant;
import java.time.ZoneOffset;
import java.util.List;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

class HardwareValidationSuiteTest {
    @Test
    @Tag("extended")
    void suiteGeneratesDeterministicPassFailResults() {
        HardwareValidationSuite.Inputs inputs = new HardwareValidationSuite.Inputs();
        inputs.driverStationDisabled = () -> true;
        inputs.rioBrownout = () -> false;
        inputs.rioBatteryVoltage = () -> 12.1;
        inputs.visionConfigOk = () -> false;
        inputs.visionFmapTagCount = () -> 0;

        Clock fixedClock = Clock.fixed(Instant.parse("2026-03-05T10:00:00Z"), ZoneOffset.UTC);
        HardwareValidationSuite suite = new HardwareValidationSuite(inputs, fixedClock, "abc123");

        List<HardwareValidationResult> results = suite.runAllChecks();
        assertEquals(4, results.size());
        assertEquals(HardwareValidationResult.Status.PASS, results.get(0).getStatus());
        assertEquals(HardwareValidationResult.Status.PASS, results.get(1).getStatus());
        assertEquals(HardwareValidationResult.Status.FAIL, results.get(2).getStatus());
        assertEquals(HardwareValidationResult.Status.FAIL, results.get(3).getStatus());
        assertTrue(results.get(0).toArtifactLine().contains("abc123"));
    }

    @Test
    @Tag("extended")
    void motorChannelCheckIncludesDirectionEncoderCurrent() {
        HardwareValidationSuite suite = new HardwareValidationSuite(
            new HardwareValidationSuite.Inputs(),
            Clock.fixed(Instant.parse("2026-03-05T10:00:00Z"), ZoneOffset.UTC),
            "hash"
        );

        HardwareValidationResult result = suite.checkMotorChannel("FrontLeft", true, false, true);
        assertEquals(HardwareValidationResult.Status.FAIL, result.getStatus());
        assertTrue(result.getDetails().contains("encoder=false"));
    }
}

package frc.robot.validation;

import java.time.Clock;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class HardwareValidationSuite {
    public static class Inputs {
        public BooleanSupplier driverStationDisabled = () -> true;
        public BooleanSupplier rioBrownout = () -> false;
        public DoubleSupplier rioBatteryVoltage = () -> 12.0;
        public IntSupplier visionFmapTagCount = () -> 0;
        public BooleanSupplier visionConfigOk = () -> false;
    }

    private final Inputs inputs;
    private final Clock clock;
    private final String buildHash;

    public HardwareValidationSuite(Inputs inputs, Clock clock, String buildHash) {
        this.inputs = inputs;
        this.clock = clock;
        this.buildHash = buildHash;
    }

    public List<HardwareValidationResult> runAllChecks() {
        List<HardwareValidationResult> results = new ArrayList<>();
        results.add(checkDriverStationSafety());
        results.add(checkRioHealth());
        results.add(checkVisionConfig());
        results.add(checkVisionFmapLoaded());
        return results;
    }

    public HardwareValidationResult checkMotorChannel(String channelName, boolean directionOk, boolean encoderOk, boolean currentOk) {
        boolean pass = directionOk && encoderOk && currentOk;
        String details = String.format("direction=%s encoder=%s current=%s", directionOk, encoderOk, currentOk);
        return result("Motor/" + channelName, pass, details);
    }

    public HardwareValidationResult checkDriverStationSafety() {
        boolean pass = inputs.driverStationDisabled.getAsBoolean();
        return result("DriverStation/DisabledForBench", pass, pass ? "Robot is disabled" : "Robot must be disabled");
    }

    public HardwareValidationResult checkRioHealth() {
        boolean brownout = inputs.rioBrownout.getAsBoolean();
        double battery = inputs.rioBatteryVoltage.getAsDouble();
        boolean pass = !brownout && battery >= 10.0;
        return result("RoboRIO/Health", pass, String.format("brownout=%s battery=%.2fV", brownout, battery));
    }

    public HardwareValidationResult checkVisionConfig() {
        boolean pass = inputs.visionConfigOk.getAsBoolean();
        return result("Vision/Config", pass, pass ? "Limelight config OK" : "Config mismatch");
    }

    public HardwareValidationResult checkVisionFmapLoaded() {
        int count = inputs.visionFmapTagCount.getAsInt();
        boolean pass = count > 0;
        return result("Vision/FmapLoaded", pass, "fmapTagCount=" + count);
    }

    private HardwareValidationResult result(String name, boolean pass, String details) {
        return new HardwareValidationResult(
            name,
            pass ? HardwareValidationResult.Status.PASS : HardwareValidationResult.Status.FAIL,
            details,
            Instant.now(clock),
            buildHash
        );
    }
}

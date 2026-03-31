package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashMap;
import java.util.Map;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

class GorusAltSistemiTest {
    private static class FakeVisionIO implements GorusAltSistemi.VisionIO {
        private final Map<String, Double> doubles = new HashMap<>();
        private final Map<String, Long> integers = new HashMap<>();
        private final Map<String, String> strings = new HashMap<>();
        private final Map<String, double[]> arrays = new HashMap<>();

        @Override
        public double getDouble(String key, double defaultValue) {
            return doubles.getOrDefault(key, defaultValue);
        }

        @Override
        public long getInteger(String key, long defaultValue) {
            return integers.getOrDefault(key, defaultValue);
        }

        @Override
        public String getString(String key, String defaultValue) {
            return strings.getOrDefault(key, defaultValue);
        }

        @Override
        public double[] getDoubleArray(String key, double[] defaultValue) {
            return arrays.getOrDefault(key, defaultValue);
        }

        @Override
        public void setNumber(String key, double value) {
            doubles.put(key, value);
        }

        @Override
        public void setDoubleArray(String key, double[] value) {
            arrays.put(key, value);
        }
    }

    private static class FakeRuntimeIO implements GorusAltSistemi.RuntimeIO {
        private boolean isReal = true;
        private boolean isRedAlliance = false;
        private double nowSec = 100.0;

        @Override
        public boolean isReal() {
            return isReal;
        }

        @Override
        public double nowSec() {
            return nowSec;
        }

        @Override
        public boolean isRedAlliance() {
            return isRedAlliance;
        }
    }

    @Test
    @Tag("fast")
    void limelightConfigStatusRequiresPipelineModeAndFmap() {
        FakeVisionIO io = new FakeVisionIO();
        FakeRuntimeIO runtime = new FakeRuntimeIO();
        GorusAltSistemi vision = new GorusAltSistemi(io, runtime);

        io.doubles.put("pipeline", 0.0);
        io.doubles.put("camMode", 0.0);
        io.doubles.put("ledMode", 0.0);
        io.integers.put("fmap/tagCount", 0L);

        assertFalse(vision.isLimelightConfigOk());
        assertTrue(vision.getLimelightConfigStatus().contains("FMAP"));

        io.integers.put("fmap/tagCount", 32L);
        assertTrue(vision.isLimelightConfigOk());
        assertEquals("OK", vision.getLimelightConfigStatus());
    }

}

package frc.robot.validation;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.List;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

class HardwareValidationArtifactWriterTest {
    @Test
    @Tag("extended")
    void writerPersistsArtifactLines() throws Exception {
        Path temp = Files.createTempDirectory("hw-validation").resolve("results.txt");
        HardwareValidationResult result = new HardwareValidationResult(
            "Vision/Config",
            HardwareValidationResult.Status.PASS,
            "ok",
            Instant.parse("2026-03-05T10:00:00Z"),
            "hash123"
        );

        HardwareValidationArtifactWriter.write(temp, List.of(result));
        String content = Files.readString(temp);
        assertTrue(content.contains("PASS"));
        assertTrue(content.contains("hash123"));
    }
}

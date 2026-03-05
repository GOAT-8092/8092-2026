package frc.robot.validation;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Collectors;

public final class HardwareValidationArtifactWriter {
    private HardwareValidationArtifactWriter() {}

    public static void write(Path outputPath, List<HardwareValidationResult> results) throws IOException {
        List<String> lines = results.stream()
            .map(HardwareValidationResult::toArtifactLine)
            .collect(Collectors.toList());
        Files.createDirectories(outputPath.getParent());
        Files.write(outputPath, lines, StandardCharsets.UTF_8);
    }
}

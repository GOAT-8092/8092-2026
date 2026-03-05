package frc.robot.validation;

import java.time.Instant;
import java.util.Objects;

public class HardwareValidationResult {
    public enum Status {
        PASS,
        FAIL
    }

    private final String checkName;
    private final Status status;
    private final String details;
    private final Instant timestamp;
    private final String robotBuildHash;

    public HardwareValidationResult(
        String checkName,
        Status status,
        String details,
        Instant timestamp,
        String robotBuildHash
    ) {
        this.checkName = Objects.requireNonNull(checkName);
        this.status = Objects.requireNonNull(status);
        this.details = details == null ? "" : details;
        this.timestamp = Objects.requireNonNull(timestamp);
        this.robotBuildHash = robotBuildHash == null ? "unknown" : robotBuildHash;
    }

    public String getCheckName() {
        return checkName;
    }

    public Status getStatus() {
        return status;
    }

    public String getDetails() {
        return details;
    }

    public Instant getTimestamp() {
        return timestamp;
    }

    public String getRobotBuildHash() {
        return robotBuildHash;
    }

    public String toArtifactLine() {
        return String.format(
            "%s | %s | %s | %s | %s",
            status.name(),
            checkName,
            timestamp.toString(),
            robotBuildHash,
            details
        );
    }
}

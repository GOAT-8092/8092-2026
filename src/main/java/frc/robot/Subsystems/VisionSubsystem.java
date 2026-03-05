package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    interface VisionIO {
        double getDouble(String key, double defaultValue);
        long getInteger(String key, long defaultValue);
        String getString(String key, String defaultValue);
        double[] getDoubleArray(String key, double[] defaultValue);
        void setNumber(String key, double value);
    }

    interface RuntimeIO {
        boolean isReal();
        double nowSec();
        boolean isRedAlliance();
    }

    static class NetworkTableVisionIO implements VisionIO {
        private final NetworkTable table;

        NetworkTableVisionIO(String tableName) {
            table = NetworkTableInstance.getDefault().getTable(tableName);
        }

        private NetworkTableEntry entry(String key) {
            return table.getEntry(key);
        }

        @Override
        public double getDouble(String key, double defaultValue) {
            return entry(key).getDouble(defaultValue);
        }

        @Override
        public long getInteger(String key, long defaultValue) {
            return entry(key).getInteger(defaultValue);
        }

        @Override
        public String getString(String key, String defaultValue) {
            return entry(key).getString(defaultValue);
        }

        @Override
        public double[] getDoubleArray(String key, double[] defaultValue) {
            return entry(key).getDoubleArray(defaultValue);
        }

        @Override
        public void setNumber(String key, double value) {
            entry(key).setNumber(value);
        }
    }

    static class WpiRuntimeIO implements RuntimeIO {
        @Override
        public boolean isReal() {
            return RobotBase.isReal();
        }

        @Override
        public double nowSec() {
            return Timer.getFPGATimestamp();
        }

        @Override
        public boolean isRedAlliance() {
            return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        }
    }

    private final VisionIO io;
    private final RuntimeIO runtime;

    private boolean cachedHasTarget = false;
    private VisionResult cachedVisionResult = new VisionResult();
    private int visionUpdateCounter = 0;
    private static final int VISION_UPDATE_RATE = 5;
    private double lastConfigApplyTimestampSec = -1.0;
    private boolean tuningInitialized = false;

    public static class VisionResult {
        public Pose2d robotPose;
        public int tagId;
        public double ambiguity;
        public double timestamp;
        public boolean valid;
    }

    public VisionSubsystem() {
        this(new NetworkTableVisionIO(VisionConstants.LIMELIGHT_NAME), new WpiRuntimeIO());
    }

    VisionSubsystem(VisionIO io, RuntimeIO runtime) {
        this.io = io;
        this.runtime = runtime;
        initializeTuningDashboard();
        applyDesiredLimelightConfig();
    }

    public boolean hasTarget() {
        return cachedHasTarget;
    }

    public double getHorizontalOffset() {
        return io.getDouble("tx", 0);
    }

    public double getVerticalOffset() {
        return io.getDouble("ty", 0);
    }

    public double getTargetArea() {
        return io.getDouble("ta", 0);
    }

    public double getTargetSkew() {
        return io.getDouble("ts", 0);
    }

    public double getDistanceToTarget() {
        double targetOffsetAngleVertical = getVerticalOffset();
        double angleToGoalDegrees = VisionConstants.LIMELIGHT_ANGLE + targetOffsetAngleVertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        double distanceInches =
            (VisionConstants.TARGET_HEIGHT - VisionConstants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);
        return distanceInches * 0.0254;
    }

    public int getTagId() {
        if (!hasTarget()) {
            return 0;
        }
        return (int) io.getInteger("tid", 0);
    }

    public int getFmapTagCount() {
        return (int) io.getInteger("fmap/tagCount", 0);
    }

    public String getFmapStatus() {
        int tagCount = getFmapTagCount();
        String fmapSize = io.getString("fmap/size", "Unknown");
        return String.format("FMAP: %d tags, Size: %s", tagCount, fmapSize);
    }

    public int getCurrentPipeline() {
        return (int) io.getDouble("pipeline", -1);
    }

    public int getCurrentCamMode() {
        return (int) io.getDouble("camMode", -1);
    }

    public int getCurrentLedMode() {
        return (int) io.getDouble("ledMode", -1);
    }

    public int getCurrentStreamMode() {
        return (int) io.getDouble("stream", -1);
    }

    private void initializeTuningDashboard() {
        SmartDashboard.putBoolean("Vision/Tune/Enable", false);
        SmartDashboard.putNumber("Vision/Tune/Pipeline", VisionConstants.DESIRED_PIPELINE);
        SmartDashboard.putNumber("Vision/Tune/SourceImageCamMode", VisionConstants.DESIRED_CAM_MODE);
        SmartDashboard.putNumber("Vision/Tune/LEDMode", VisionConstants.DESIRED_LED_MODE);
        SmartDashboard.putNumber("Vision/Tune/StreamMode", VisionConstants.DESIRED_STREAM_MODE);
        SmartDashboard.putString("Vision/Tune/Unsupported", "Resolution, Stream Orientation, Exposure, LED Power -> Limelight UI");
        tuningInitialized = true;
    }

    private int getDesiredPipeline() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/Pipeline", VisionConstants.DESIRED_PIPELINE);
        }
        return VisionConstants.DESIRED_PIPELINE;
    }

    private int getDesiredCamMode() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/SourceImageCamMode", VisionConstants.DESIRED_CAM_MODE);
        }
        return VisionConstants.DESIRED_CAM_MODE;
    }

    private int getDesiredLedMode() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/LEDMode", VisionConstants.DESIRED_LED_MODE);
        }
        return VisionConstants.DESIRED_LED_MODE;
    }

    private int getDesiredStreamMode() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/StreamMode", VisionConstants.DESIRED_STREAM_MODE);
        }
        return VisionConstants.DESIRED_STREAM_MODE;
    }

    public void applyDesiredLimelightConfig() {
        io.setNumber("pipeline", getDesiredPipeline());
        io.setNumber("camMode", getDesiredCamMode());
        io.setNumber("ledMode", getDesiredLedMode());
        io.setNumber("stream", getDesiredStreamMode());
        lastConfigApplyTimestampSec = runtime.nowSec();
    }

    public boolean isLimelightConfigOk() {
        boolean pipelineOk = getCurrentPipeline() == getDesiredPipeline();
        boolean camModeOk = getCurrentCamMode() == getDesiredCamMode();
        boolean ledModeOk = getCurrentLedMode() == getDesiredLedMode();
        boolean streamOk = getCurrentStreamMode() == getDesiredStreamMode();
        boolean fmapLoaded = getFmapTagCount() > 0;
        return pipelineOk && camModeOk && ledModeOk && streamOk && fmapLoaded;
    }

    public String getLimelightConfigStatus() {
        int desiredPipeline = getDesiredPipeline();
        int desiredCamMode = getDesiredCamMode();
        int desiredLedMode = getDesiredLedMode();
        int desiredStreamMode = getDesiredStreamMode();

        boolean pipelineOk = getCurrentPipeline() == desiredPipeline;
        boolean camModeOk = getCurrentCamMode() == desiredCamMode;
        boolean ledModeOk = getCurrentLedMode() == desiredLedMode;
        boolean streamOk = getCurrentStreamMode() == desiredStreamMode;
        int fmapTagCount = getFmapTagCount();

        if (!pipelineOk) {
            return String.format("Pipeline mismatch (%d != %d)", getCurrentPipeline(), desiredPipeline);
        }
        if (!camModeOk) {
            return String.format("CamMode mismatch (%d != %d)", getCurrentCamMode(), desiredCamMode);
        }
        if (!ledModeOk) {
            return String.format("LedMode mismatch (%d != %d)", getCurrentLedMode(), desiredLedMode);
        }
        if (!streamOk) {
            return String.format("Stream mismatch (%d != %d)", getCurrentStreamMode(), desiredStreamMode);
        }
        if (fmapTagCount <= 0) {
            return "FMAP not loaded (tagCount=0)";
        }
        if (fmapTagCount < VisionConstants.TOTAL_APRILTAGS) {
            return String.format("FMAP partial (%d/%d tags)", fmapTagCount, VisionConstants.TOTAL_APRILTAGS);
        }
        return "OK";
    }

    private void updateCachedVisionData() {
        cachedHasTarget = io.getDouble("tv", 0) == 1;
        visionUpdateCounter++;
        if (visionUpdateCounter >= VISION_UPDATE_RATE) {
            visionUpdateCounter = 0;
            if (cachedHasTarget) {
                cachedVisionResult = getRobotPoseFromAprilTagInternal();
            } else {
                cachedVisionResult.valid = false;
            }
        }
    }

    private VisionResult getRobotPoseFromAprilTagInternal() {
        VisionResult result = new VisionResult();
        result.valid = false;

        // Debug: Check raw tv value
        double tv = io.getDouble("tv", 0);
        SmartDashboard.putBoolean("Vision/Debug/RawTV", tv == 1);

        // BUG FIX: Check raw tv value, not cached value!
        if (tv != 1) {
            SmartDashboard.putString("Vision/Debug/Status", "No target (tv=" + tv + ")");
            System.out.println("Vision: No target detected (tv=" + tv + ")");
            return result;
        }

        System.out.println("Vision: Target detected! Reading pose data...");

        String poseEntry = runtime.isRedAlliance() ? "botpose_wpired" : "botpose_wpiblue";
        SmartDashboard.putString("Vision/Debug/PoseEntry", poseEntry);
        System.out.println("Vision: Reading from " + poseEntry);

        double[] botPoseArray = io.getDoubleArray(poseEntry, new double[0]);
        SmartDashboard.putNumber("Vision/Debug/PoseArrayLength", botPoseArray.length);
        System.out.println("Vision: botpose array length = " + botPoseArray.length);

        if (botPoseArray.length < 7) {
            SmartDashboard.putString("Vision/Debug/Status", "Array too short: " + botPoseArray.length);
            System.out.println("Vision: ERROR - Array too short! Expected 7, got " + botPoseArray.length);
            return result;
        }

        double x = botPoseArray[0];
        double y = botPoseArray[1];
        double yaw = botPoseArray[5];
        double latency = botPoseArray[6] / 1000.0;
        double timestamp = runtime.nowSec() - latency;

        System.out.println(String.format("Vision: Got pose - X:%.2f Y:%.2f Yaw:%.1f Latency:%.3f",
            x, y, yaw, latency));

        result.robotPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
        result.timestamp = timestamp;
        result.tagId = (int) io.getInteger("tid", 0);

        double[] targetPoseArray = io.getDoubleArray("targetpose_cameraspace", new double[0]);
        result.ambiguity = (targetPoseArray.length >= 7) ? targetPoseArray[6] : 1.0;

        // Debug values
        SmartDashboard.putNumber("Vision/Debug/TagID", result.tagId);
        SmartDashboard.putNumber("Vision/Debug/RobotX", x);
        SmartDashboard.putNumber("Vision/Debug/RobotY", y);
        SmartDashboard.putNumber("Vision/Debug/Ambiguity", result.ambiguity);

        // Validate result
        result.valid = (result.ambiguity < VisionConstants.AMBIGUITY_THRESHOLD) && (result.tagId > 0);

        SmartDashboard.putBoolean("Vision/Debug/Valid", result.valid);
        SmartDashboard.putString("Vision/Debug/Status", result.valid ? "OK" : "Invalid");

        System.out.println("Vision: Result valid=" + result.valid + " (ambiguity=" + result.ambiguity + ", tagID=" + result.tagId + ")");

        return result;
    }

    @Override
    public void periodic() {
        if (runtime.isReal()) {
            if (!tuningInitialized) {
                initializeTuningDashboard();
            }

            double nowSec = runtime.nowSec();
            if (lastConfigApplyTimestampSec < 0
                || nowSec - lastConfigApplyTimestampSec >= VisionConstants.CONFIG_REAPPLY_INTERVAL_SEC) {
                applyDesiredLimelightConfig();
            }

            updateCachedVisionData();

            // NetworkTables connectivity test
            SmartDashboard.putBoolean("Vision/NT/Connected", io.getDouble("tv", -999) != -999);

            SmartDashboard.putBoolean("Vision/HasTarget", cachedHasTarget);
            if (cachedHasTarget) {
                SmartDashboard.putNumber("Vision/HorizontalOffset", io.getDouble("tx", 0));
                SmartDashboard.putNumber("Vision/VerticalOffset", io.getDouble("ty", 0));
                SmartDashboard.putNumber("Vision/TargetArea", io.getDouble("ta", 0));
                SmartDashboard.putNumber("Vision/DistanceToTarget", getDistanceToTarget());
            }
            SmartDashboard.putNumber("Vision/FmapTagCount", getFmapTagCount());
            SmartDashboard.putString("Vision/FmapStatus", getFmapStatus());
            SmartDashboard.putNumber("Vision/Pipeline", getCurrentPipeline());
            SmartDashboard.putNumber("Vision/CamMode", getCurrentCamMode());
            SmartDashboard.putNumber("Vision/LEDMode", getCurrentLedMode());
            SmartDashboard.putNumber("Vision/StreamMode", getCurrentStreamMode());
            SmartDashboard.putBoolean("Vision/ConfigOk", isLimelightConfigOk());
            SmartDashboard.putString("Vision/ConfigStatus", getLimelightConfigStatus());

            SmartDashboard.putBoolean("Vision/PoseValid", cachedVisionResult.valid);
            if (cachedVisionResult.valid) {
                SmartDashboard.putNumber("Vision/RobotX", cachedVisionResult.robotPose.getX());
                SmartDashboard.putNumber("Vision/RobotY", cachedVisionResult.robotPose.getY());
                SmartDashboard.putNumber("Vision/RobotYaw", cachedVisionResult.robotPose.getRotation().getDegrees());
                SmartDashboard.putNumber("Vision/TagID", cachedVisionResult.tagId);
                SmartDashboard.putNumber("Vision/Ambiguity", cachedVisionResult.ambiguity);
            }
        } else {
            SmartDashboard.putBoolean("Vision/HasTarget", true);
            SmartDashboard.putNumber("Vision/HorizontalOffset", 5.0);
            SmartDashboard.putNumber("Vision/DistanceToTarget", 3.0);
            SmartDashboard.putBoolean("Vision/PoseValid", true);
            SmartDashboard.putNumber("Vision/RobotX", 2.0);
            SmartDashboard.putNumber("Vision/RobotY", 3.0);
            SmartDashboard.putNumber("Vision/RobotYaw", 0.0);
            SmartDashboard.putNumber("Vision/TagID", 1);
            SmartDashboard.putString("Vision/Mode", "Simulation");
        }
    }

    public VisionResult getRobotPoseFromAprilTag() {
        // For debugging: Always read fresh data, don't use cache
        return getRobotPoseFromAprilTagInternal();
    }
}

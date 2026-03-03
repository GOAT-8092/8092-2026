package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private NetworkTable limelightTable;

    // Cached NetworkTable entries for performance
    private final Object tvEntry;
    private final Object txEntry;
    private final Object tyEntry;
    private final Object taEntry;
    private final Object tidEntry;
    private final Object botPoseEntry;
    private final Object targetPoseEntry;
    private final Object fmapTagCountEntry;
    private final Object fmapSizeEntry;

    // Performance optimization: cache expensive operations
    private boolean cachedHasTarget = false;
    private VisionSubsystem.VisionResult cachedVisionResult = new VisionSubsystem.VisionResult();
    private int visionUpdateCounter = 0;
    private static final int VISION_UPDATE_RATE = 5;  // Update pose every 5 cycles (100ms)

    /**
     * Result from AprilTag pose estimation
     */
    public static class VisionResult {
        public Pose2d robotPose;
        public int tagId;
        public double ambiguity;
        public double timestamp;
        public boolean valid;
    }

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAME);

        // Cache NetworkTable entries for better performance
        tvEntry = limelightTable.getEntry("tv");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        taEntry = limelightTable.getEntry("ta");
        tidEntry = limelightTable.getEntry("tid");
        botPoseEntry = limelightTable.getEntry("botpose_wpiblue");
        targetPoseEntry = limelightTable.getEntry("targetpose_cameraspace");
        fmapTagCountEntry = limelightTable.getEntry("fmap/tagCount");
        fmapSizeEntry = limelightTable.getEntry("fmap/size");
    }

    public boolean hasTarget() {
        return cachedHasTarget;
    }

    public double getHorizontalOffset() {
        return ((edu.wpi.first.networktables.NetworkTableEntry)txEntry).getDouble(0);
    }

    public double getVerticalOffset() {
        return ((edu.wpi.first.networktables.NetworkTableEntry)tyEntry).getDouble(0);
    }

    public double getTargetArea() {
        return ((edu.wpi.first.networktables.NetworkTableEntry)taEntry).getDouble(0);
    }

    public double getTargetSkew() {
        return limelightTable.getEntry("ts").getDouble(0);
    }

    public double getDistanceToTarget() {
        double targetOffsetAngle_Vertical = getVerticalOffset();

        double angleToGoalDegrees = VisionConstants.LIMELIGHT_ANGLE + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        double distanceFromLimelightToGoalInches = (VisionConstants.TARGET_HEIGHT - VisionConstants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches * 0.0254; // Convert to meters
    }

    /**
     * Get the ID of the currently detected AprilTag
     * @return Tag ID, or 0 if no target detected
     */
    public int getTagId() {
        if (!hasTarget()) {
            return 0;
        }
        return (int) ((edu.wpi.first.networktables.NetworkTableEntry)tidEntry).getInteger(0);
    }

    /**
     * Check if Limelight has a valid FMAP loaded
     * @return Number of AprilTags in FMAP, or 0 if not loaded
     */
    public int getFmapTagCount() {
        // Limelight publishes the number of tags in its loaded FMAP
        return (int) ((edu.wpi.first.networktables.NetworkTableEntry)fmapTagCountEntry).getInteger(0);
    }

    /**
     * Get FMAP status information for debugging
     * @return Status string with FMAP info
     */
    public String getFmapStatus() {
        int tagCount = getFmapTagCount();
        String fmapSize = ((edu.wpi.first.networktables.NetworkTableEntry)fmapSizeEntry).getString("Unknown");
        return String.format("FMAP: %d tags, Size: %s", tagCount, fmapSize);
    }

    /**
     * Internal method to update cached vision data
     * Call this periodically instead of every cycle
     */
    private void updateCachedVisionData() {
        // Update hasTarget cache
        cachedHasTarget = ((edu.wpi.first.networktables.NetworkTableEntry)tvEntry).getDouble(0) == 1;

        // Only update pose estimation every N cycles to save performance
        visionUpdateCounter++;
        if (visionUpdateCounter >= VISION_UPDATE_RATE) {
            visionUpdateCounter = 0;

            if (cachedHasTarget) {
                // Get full pose estimation (expensive operation)
                cachedVisionResult = getRobotPoseFromAprilTagInternal();
            } else {
                cachedVisionResult.valid = false;
            }
        }
    }

    /**
     * Internal method that doesn't use cache
     */
    private VisionResult getRobotPoseFromAprilTagInternal() {
        VisionResult result = new VisionResult();
        result.valid = false;

        if (!cachedHasTarget) {
            return result;
        }

        // Determine which pose array to use based on alliance
        boolean isRedAlliance = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        String poseEntry = isRedAlliance ? "botpose_wpired" : "botpose_wpiblue";
        double[] botPoseArray = limelightTable.getEntry(poseEntry).getDoubleArray(new double[0]);

        // botpose array: [x, y, z, roll, pitch, yaw, total_latency]
        if (botPoseArray.length < 7) {
            return result;
        }

        // Extract pose data
        double x = botPoseArray[0];
        double y = botPoseArray[1];
        double yaw = botPoseArray[5];
        double latency = botPoseArray[6] / 1000.0; // Convert ms to seconds
        double timestamp = Timer.getFPGATimestamp() - latency;

        result.robotPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
        result.timestamp = timestamp;
        result.tagId = (int) ((edu.wpi.first.networktables.NetworkTableEntry)tidEntry).getInteger(0);

        // Get ambiguity from targetpose_cameraspace array
        // [x, y, z, roll, pitch, yaw, ambiguity]
        double[] targetPoseArray = limelightTable.getEntry("targetpose_cameraspace")
            .getDoubleArray(new double[0]);
        result.ambiguity = (targetPoseArray.length >= 7) ? targetPoseArray[6] : 1.0;

        // Validate result
        result.valid = (result.ambiguity < VisionConstants.AMBIGUITY_THRESHOLD) && (result.tagId > 0);

        return result;
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            // Update cached vision data (including expensive pose estimation)
            updateCachedVisionData();

            // Use cached values for SmartDashboard
            SmartDashboard.putBoolean("Vision/HasTarget", cachedHasTarget);

            // Only query these if we have a target (optimization)
            if (cachedHasTarget) {
                SmartDashboard.putNumber("Vision/HorizontalOffset",
                    ((edu.wpi.first.networktables.NetworkTableEntry)txEntry).getDouble(0));
                SmartDashboard.putNumber("Vision/VerticalOffset",
                    ((edu.wpi.first.networktables.NetworkTableEntry)tyEntry).getDouble(0));
                SmartDashboard.putNumber("Vision/TargetArea",
                    ((edu.wpi.first.networktables.NetworkTableEntry)taEntry).getDouble(0));
                SmartDashboard.putNumber("Vision/DistanceToTarget", getDistanceToTarget());
            }

            // FMAP status (these rarely change, update every cycle is fine)
            SmartDashboard.putNumber("Vision/FmapTagCount", getFmapTagCount());
            SmartDashboard.putString("Vision/FmapStatus", getFmapStatus());

            // Pose estimation data from cache (updated every VISION_UPDATE_RATE cycles)
            SmartDashboard.putBoolean("Vision/PoseValid", cachedVisionResult.valid);
            if (cachedVisionResult.valid) {
                SmartDashboard.putNumber("Vision/RobotX", cachedVisionResult.robotPose.getX());
                SmartDashboard.putNumber("Vision/RobotY", cachedVisionResult.robotPose.getY());
                SmartDashboard.putNumber("Vision/RobotYaw", cachedVisionResult.robotPose.getRotation().getDegrees());
                SmartDashboard.putNumber("Vision/TagID", cachedVisionResult.tagId);
                SmartDashboard.putNumber("Vision/Ambiguity", cachedVisionResult.ambiguity);
            }
        } else {
            // Simulation - mock vision data
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

    /**
     * Public method that returns current vision result (from cache for performance)
     */
    public VisionResult getRobotPoseFromAprilTag() {
        return cachedVisionResult;
    }
}
package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.AprilTagFieldLayout;

/**
 * Aligns the robot to face and drive toward a specific AprilTag.
 * Uses vision feedback to position the robot in front of the tag.
 */
public class AlignToAprilTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    // Target configuration
    private final int targetTagId;
    private final double targetDistanceMeters;  // How far in front of tag to stop
    private final double toleranceMeters;        // Position tolerance
    private final double toleranceDegrees;      // Heading tolerance

    // PID controllers for holonomic alignment
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    // Alliance configuration
    private boolean isRedAlliance;

    /**
     * Creates a new AlignToAprilTagCommand.
     *
     * @param driveSubsystem The drive subsystem to use
     * @param visionSubsystem The vision subsystem to use
     * @param targetTagId The AprilTag ID to align to (1-16)
     * @param targetDistanceMeters How far in front of the tag to stop (default: 1.0m)
     */
    public AlignToAprilTagCommand(
            DriveSubsystem driveSubsystem,
            VisionSubsystem visionSubsystem,
            int targetTagId,
            double targetDistanceMeters) {

        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetTagId = targetTagId;
        this.targetDistanceMeters = targetDistanceMeters;
        this.toleranceMeters = 0.15;  // 15cm tolerance
        this.toleranceDegrees = 5.0;    // 5 degree heading tolerance

        // Create PID controllers
        // These values might need tuning based on robot performance
        xController = new PIDController(2.0, 0, 0.1);    // Forward/back
        yController = new PIDController(2.0, 0, 0.1);    // Strafe
        thetaController = new PIDController(1.5, 0, 0.05); // Rotation

        // Enable continuous input for theta (handles wraparound at -180/180)
        thetaController.enableContinuousInput(-180, 180);

        // Set tolerances
        xController.setTolerance(toleranceMeters);
        yController.setTolerance(toleranceMeters);
        thetaController.setTolerance(toleranceDegrees);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("===========================================");
        System.out.println("ALIGN TO APRILTAG COMMAND INITIALIZED");
        System.out.println("Target Tag ID: " + targetTagId);
        System.out.println("Target Distance: " + targetDistanceMeters + " meters");
        System.out.println("===========================================");

        // Determine alliance at command start
        isRedAlliance = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        SmartDashboard.putString("AlignToAprilTag/Status", "Initializing");
        SmartDashboard.putNumber("AlignToAprilTag/TargetTag", targetTagId);
        SmartDashboard.putBoolean("AlignToAprilTag/IsRed", isRedAlliance);
    }

    @Override
    public void execute() {
        // Get vision result
        VisionSubsystem.VisionResult result = visionSubsystem.getRobotPoseFromAprilTag();

        SmartDashboard.putBoolean("AlignToAprilTag/ResultValid", result.valid);
        SmartDashboard.putNumber("AlignToAprilTag/ResultTagID", result.tagId);

        // If no valid vision target, stop
        if (!result.valid) {
            SmartDashboard.putString("AlignToAprilTag/Status", "No valid vision");
            System.out.println("AlignToAprilTag: No valid vision - stopping");
            driveSubsystem.stopAllMotors();
            return;
        }

        if (result.tagId != targetTagId) {
            SmartDashboard.putString("AlignToAprilTag/Status", "Wrong tag: " + result.tagId);
            System.out.println("AlignToAprilTag: Wrong tag ID " + result.tagId + " (want " + targetTagId + ")");
            driveSubsystem.stopAllMotors();
            return;
        }

        // Get target tag pose from field layout
        Pose2d targetPose = AprilTagFieldLayout.getTagPose(targetTagId, isRedAlliance);
        if (targetPose == null) {
            SmartDashboard.putString("AlignToAprilTag/Status", "Tag pose not found");
            System.out.println("AlignToAprilTag: Tag pose not found for ID " + targetTagId);
            driveSubsystem.stopAllMotors();
            return;
        }

        // Current robot pose from vision
        Pose2d currentPose = result.robotPose;
        System.out.println(String.format("AlignToAprilTag: Robot at (%.2f, %.2f, %.1f°), Tag at (%.2f, %.2f)",
            currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees(),
            targetPose.getX(), targetPose.getY()));

        // Calculate desired position (in front of tag, facing tag)
        // Target position = tag position - offset in direction tag is facing
        double tagYaw = targetPose.getRotation().getRadians();
        double targetX = targetPose.getX() - Math.cos(tagYaw) * targetDistanceMeters;
        double targetY = targetPose.getY() - Math.sin(tagYaw) * targetDistanceMeters;

        // Calculate errors
        double xError = targetX - currentPose.getX();
        double yError = targetY - currentPose.getY();

        // For heading, we want to face the tag
        // Calculate required heading to face the tag
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double desiredYaw = Math.atan2(dy, dx);

        double currentYaw = currentPose.getRotation().getRadians();
        double thetaError = desiredYaw - currentYaw;

        // Normalize theta error to -PI to PI
        while (thetaError > Math.PI) thetaError -= 2 * Math.PI;
        while (thetaError < -Math.PI) thetaError += 2 * Math.PI;

        // Debug output
        SmartDashboard.putNumber("AlignToAprilTag/XError", xError);
        SmartDashboard.putNumber("AlignToAprilTag/YError", yError);
        SmartDashboard.putNumber("AlignToAprilTag/YawError", Math.toDegrees(thetaError));

        // Calculate drive outputs using PID
        double xSpeed = xController.calculate(xError, 0);
        double ySpeed = yController.calculate(yError, 0);
        double thetaSpeed = thetaController.calculate(Math.toDegrees(thetaError), 0);

        SmartDashboard.putNumber("AlignToAprilTag/XSpeed", xSpeed);
        SmartDashboard.putNumber("AlignToAprilTag/YSpeed", ySpeed);
        SmartDashboard.putNumber("AlignToAprilTag/ThetaSpeed", thetaSpeed);
        SmartDashboard.putString("AlignToAprilTag/Status", "Driving");

        // Drive the robot
        // Note: Using field-oriented drive for better control
        driveSubsystem.drive(ySpeed, xSpeed, thetaSpeed);
    }

    @Override
    public boolean isFinished() {
        // Check if we're at the target position
        VisionSubsystem.VisionResult result = visionSubsystem.getRobotPoseFromAprilTag();

        if (!result.valid || result.tagId != targetTagId) {
            return false;  // Don't finish if we can't see the target
        }

        Pose2d targetPose = AprilTagFieldLayout.getTagPose(targetTagId, isRedAlliance);
        if (targetPose == null) {
            return false;
        }

        Pose2d currentPose = result.robotPose;

        // Calculate distance to target position
        double tagYaw = targetPose.getRotation().getRadians();
        double targetX = targetPose.getX() - Math.cos(tagYaw) * targetDistanceMeters;
        double targetY = targetPose.getY() - Math.sin(tagYaw) * targetDistanceMeters;

        double distanceError = Math.sqrt(
            Math.pow(targetX - currentPose.getX(), 2) +
            Math.pow(targetY - currentPose.getY(), 2)
        );

        // Check if we're at tolerance
        boolean atPosition = distanceError < toleranceMeters;

        // Check if we're facing the right direction
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double desiredYaw = Math.atan2(dy, dx);
        double headingError = Math.abs(desiredYaw - currentPose.getRotation().getRadians());
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        headingError = Math.abs(headingError);

        boolean atHeading = Math.toDegrees(headingError) < toleranceDegrees;

        boolean finished = atPosition && atHeading;
        if (finished) {
            System.out.println("AlignToAprilTag: ALIGNMENT COMPLETE!");
            System.out.println("  Position tolerance met: " + atPosition);
            System.out.println("  Heading tolerance met: " + atHeading);
        }

        return atPosition && atHeading;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("===========================================");
        System.out.println("ALIGN TO APRILTAG COMMAND ENDING");
        System.out.println("Interrupted: " + interrupted);
        System.out.println("===========================================");
        driveSubsystem.stopAllMotors();
        SmartDashboard.putString("AlignToAprilTag/Status", interrupted ? "Interrupted" : "Finished");
    }
}

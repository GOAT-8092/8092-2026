package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
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
        // Determine alliance at command start
        isRedAlliance = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }

    @Override
    public void execute() {
        // Get vision result
        VisionSubsystem.VisionResult result = visionSubsystem.getRobotPoseFromAprilTag();

        // If no valid vision target, stop
        if (!result.valid || result.tagId != targetTagId) {
            driveSubsystem.stopAllMotors();
            return;
        }

        // Get target tag pose from field layout
        Pose2d targetPose = AprilTagFieldLayout.getTagPose(targetTagId, isRedAlliance);
        if (targetPose == null) {
            driveSubsystem.stopAllMotors();
            return;
        }

        // Current robot pose from vision
        Pose2d currentPose = result.robotPose;

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

        // Calculate drive outputs using PID
        double xSpeed = xController.calculate(xError, 0);
        double ySpeed = yController.calculate(yError, 0);
        double thetaSpeed = thetaController.calculate(Math.toDegrees(thetaError), 0);

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

        return atPosition && atHeading;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopAllMotors();
    }
}

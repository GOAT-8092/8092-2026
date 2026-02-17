package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class VisionSubsystem extends SubsystemBase {
    private NetworkTable limelightTable;

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable(ModuleConstants.LIMELIGHT_NAME);
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    public double getHorizontalOffset() {
        return limelightTable.getEntry("tx").getDouble(0);
    }

    public double getVerticalOffset() {
        return limelightTable.getEntry("ty").getDouble(0);
    }

    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0);
    }

    public double getTargetSkew() {
        return limelightTable.getEntry("ts").getDouble(0);
    }

    public double getDistanceToTarget() {
        double targetOffsetAngle_Vertical = getVerticalOffset();

        double angleToGoalDegrees = ModuleConstants.LIMELIGHT_ANGLE + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        double distanceFromLimelightToGoalInches = (ModuleConstants.TARGET_HEIGHT - ModuleConstants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches * 0.0254; // Convert to meters
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            SmartDashboard.putBoolean("Has Target", hasTarget());
            SmartDashboard.putNumber("Horizontal Offset", getHorizontalOffset());
            SmartDashboard.putNumber("Distance to Target", getDistanceToTarget());
        } else {
            // Simulation - mock vision data
            SmartDashboard.putBoolean("Has Target", true);
            SmartDashboard.putNumber("Horizontal Offset", 5.0); // Mock 5 degree offset
            SmartDashboard.putNumber("Distance to Target", 3.0); // Mock 3 meters
            SmartDashboard.putString("Vision Mode", "Simulation");
        }
    }
}
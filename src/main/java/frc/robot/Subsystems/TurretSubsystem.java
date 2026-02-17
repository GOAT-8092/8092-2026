package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.MotorConstants;

public class TurretSubsystem extends SubsystemBase {
    private SparkMax turretMotor;
    private RelativeEncoder turretEncoder;

    public TurretSubsystem() {
        if (MotorConstants.ENABLE_NON_DRIVE_MOTORS) {
            turretMotor = new SparkMax(MotorConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(MotorConstants.TURRET_MOTOR_INVERTED);
            turretMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

            turretEncoder = turretMotor.getEncoder();
        } else {
            turretMotor = null;
            turretEncoder = null;
        }
    }

    public void rotate(double speed) {
        if (turretMotor != null) {
            turretMotor.set(speed);
        }
    }

    public void stop() {
        if (turretMotor != null) {
            turretMotor.set(0);
        }
    }

    public double getAngle() {
        // Assuming encoder gives position in rotations, convert to degrees
        if (turretEncoder != null) {
            return turretEncoder.getPosition() * 360.0;
        }
        return 0.0;
    }

    public void setAngle(double angle) {
        // Simple proportional control - in real implementation, use PID
        double currentAngle = getAngle();
        double error = angle - currentAngle;
        double speed = error * 0.01; // Tune this
        rotate(speed);
    }
}
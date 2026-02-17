
package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.MotorConstants;

public class ShooterSubsystem extends SubsystemBase {
    private SparkMax leftShooterMotor;
    private SparkMax rightShooterMotor;
    private SparkMax topShooterMotor;

    public ShooterSubsystem() {
        if (MotorConstants.ENABLE_NON_DRIVE_MOTORS) {
            leftShooterMotor = new SparkMax(MotorConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
            rightShooterMotor = new SparkMax(MotorConstants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
            topShooterMotor = new SparkMax(MotorConstants.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);

            SparkMaxConfig leftConfig = new SparkMaxConfig();
            leftConfig.inverted(MotorConstants.LEFT_SHOOTER_INVERTED);
            leftShooterMotor.configure(leftConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

            SparkMaxConfig rightConfig = new SparkMaxConfig();
            rightConfig.inverted(MotorConstants.RIGHT_SHOOTER_INVERTED);
            rightShooterMotor.configure(rightConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

            SparkMaxConfig topConfig = new SparkMaxConfig();
            topConfig.inverted(MotorConstants.TOP_SHOOTER_INVERTED);
            topShooterMotor.configure(topConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        } else {
            leftShooterMotor = null;
            rightShooterMotor = null;
            topShooterMotor = null;
        }
    }

    public void shoot() {
        if (leftShooterMotor != null) leftShooterMotor.set(ModuleConstants.SHOOTER_SPEED);
        if (rightShooterMotor != null) rightShooterMotor.set(ModuleConstants.SHOOTER_SPEED);
        if (topShooterMotor != null) topShooterMotor.set(ModuleConstants.SHOOTER_SPEED);
    }

    public void stop() {
        if (leftShooterMotor != null) leftShooterMotor.set(0);
        if (rightShooterMotor != null) rightShooterMotor.set(0);
        if (topShooterMotor != null) topShooterMotor.set(0);
    }
}
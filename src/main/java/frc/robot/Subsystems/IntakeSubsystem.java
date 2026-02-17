package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;

    public IntakeSubsystem() {
        if (MotorConstants.ENABLE_NON_DRIVE_MOTORS) {
            intakeMotor = new SparkMax(MotorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(MotorConstants.INTAKE_MOTOR_INVERTED);
            intakeMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        } else {
            // Stub out motor to avoid creating CAN devices when not needed
            intakeMotor = null;
        }
    }

    public void intake() {
        if (intakeMotor != null) {
            intakeMotor.set(ModuleConstants.INTAKE_SPEED);
        }
    }

    public void outtake() {
        if (intakeMotor != null) {
            intakeMotor.set(-ModuleConstants.INTAKE_SPEED);
        }
    }

    public void stop() {
        if (intakeMotor != null) {
            intakeMotor.set(0);
        }
    }
}
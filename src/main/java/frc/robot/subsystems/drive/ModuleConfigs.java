package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ModuleConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public ModuleConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = BaseFalconSwerveConstants.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = BaseFalconSwerveConstants.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = BaseFalconSwerveConstants.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = BaseFalconSwerveConstants.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = BaseFalconSwerveConstants.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = BaseFalconSwerveConstants.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = BaseFalconSwerveConstants.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = BaseFalconSwerveConstants.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = BaseFalconSwerveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = BaseFalconSwerveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = BaseFalconSwerveConstants.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = BaseFalconSwerveConstants.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = BaseFalconSwerveConstants.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = BaseFalconSwerveConstants.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = BaseFalconSwerveConstants.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = BaseFalconSwerveConstants.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = BaseFalconSwerveConstants.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = BaseFalconSwerveConstants.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = BaseFalconSwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = BaseFalconSwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = BaseFalconSwerveConstants.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = BaseFalconSwerveConstants.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = BaseFalconSwerveConstants.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = BaseFalconSwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = BaseFalconSwerveConstants.closedLoopRamp;
    } 
}


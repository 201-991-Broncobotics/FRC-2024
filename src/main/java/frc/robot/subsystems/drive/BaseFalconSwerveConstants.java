package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.robot.Constants.TuningConstants;

public final class BaseFalconSwerveConstants {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.75); // 28" width -> 22.75" track width
        public static final double wheelBase = Units.inchesToMeters(18.75); // 24" drivetrain length -> 18.75" wheel base
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 10;
        public static final int angleCurrentThreshold = 20;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 15;
        public static final int driveCurrentThreshold = 20;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = TuningConstants.drive_motor_p; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = (TuningConstants.drive_static_voltage / 12.0);
        public static final double driveKV = (TuningConstants.drive_equilibrium_voltage / 12.0);
        public static final double driveKA = (TuningConstants.drive_acceleration_voltage / 12.0);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = TuningConstants.max_linear_speed * 0.3048;
        /** Radians per Second */
        public static final double maxAngularVelocity = TuningConstants.max_angular_speed * Math.PI / 180.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
}


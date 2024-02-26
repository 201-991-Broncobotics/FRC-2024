package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final class TuningConstants { // Basically stuff you have to tune

        public static final double CANCoder0_zero = 250.75, // Front Left
                                   CANCoder1_zero = 326.51, // Front Right
                                   CANCoder2_zero = 318.16, // Back Left
                                   CANCoder3_zero = 137.98; // Back Right
        
        /* Swerve Drive Constants */

        public static final double drive_motor_p = 0.05,
                                   drive_static_voltage = 0.32, 
                                   drive_equilibrium_voltage = 1.51, 
                                   drive_acceleration_voltage = 0.27, // SYSID values: KS, KV, KA; they are automatically divided by 12 later
                                   max_linear_speed = 10, // feet per second; theoretical max is 13.5
                                   max_angular_speed = 225; // degrees per second; theoretical max is theoretical maximum is max_linear_speed * 46.6436741705 which is roughly 629.6896013018

        /* PathPlanner PID Constants */

        public static final double autonomous_max_linear_speed = 8,
                                   autonomous_ramp_up_time_linear = 2.5, // in seconds to reach max 
                                   autonomous_max_angular_speed = 180, 
                                   autonomous_ramp_up_time_angular = 1.5,
                                   autonomous_translation_p_controller = 2, 
                                   autonomous_angle_p_controller = 4;
        
        /* Driving PID Constants */

        public static final double teleop_angle_p = 0.165,
                                   teleop_angle_i = 0,
                                   teleop_angle_e = 1.35,
                                   teleop_translation_p = 0.4, 
                                   teleop_translation_i = 0,
                                   teleop_translation_e = 1;
        
        public static final double turn_slow_ratio = 2; // because slowing down rotation and translation by the same factor is insane
                                    // ex. it its 4, then 0.6 translation ratio goes to a 0.9 turning ratio
        
        /* Pivot Constants */

        public static final double pivot_p = 0.045, 
                                   pivot_i = 0, 
                                   pivot_e = 1.1;
    }

    public static final class Buttons {

        /* D-pad */

        public static final int dpad_up = 0, 
                                dpad_right = 90, 
                                dpad_down = 180, 
                                dpad_left = 270;

        /* Which is which */

        public static final int joystick_usb_port = 2, 
                                driver_usb_port = 1, 
                                operator_usb_port = 0;

        /* Driver Buttons */

        public static final int xBoxTranslationAxis = XboxController.Axis.kLeftY.value, 
                                xBoxStrafeAxis = XboxController.Axis.kLeftX.value,

                                xBoxRotationAxis = XboxController.Axis.kRightX.value,

                                xBoxDirectionXAxis = XboxController.Axis.kRightX.value, 
                                xBoxDirectionYAxis = XboxController.Axis.kRightY.value, 
                                xBoxTurnRightAxis = XboxController.Axis.kRightTrigger.value, 
                                xBoxTurnLeftAxis = XboxController.Axis.kLeftTrigger.value, 
                                
                                xBoxSlowAxis = XboxController.Axis.kRightTrigger.value, 
                                xBoxSlowButtonOne = XboxController.Button.kRightBumper.value, 
                                xBoxSlowButtonTwo = XboxController.Button.kLeftBumper.value, 

                                xBoxRobotCentricButton = XboxController.Button.kLeftBumper.value,

                                xBoxZeroGyroButton = XboxController.Button.kY.value, 
                                xBoxMakeXButton = XboxController.Button.kX.value, 
                                xBoxTerminateCommandsDriverButton = XboxController.Button.kBack.value,

                                joystickTranslationAxis = 1, // backward is positive
                                joystickStrafeAxis = 0, // right is positive

                                joystickRotationAxis = 5, // right is positive

                                joystickSlowButton = 1,

                                joystickZeroGyroButton = 3, 
                                joystickMakeXButton = 2, 
                                joystickTerminateCommandsButton = 4; // if we push the slider forward it cancels commands
                                // .getPOV() is the same for joystick; joystick is a T.Flight Hotas One

        /* Operator Buttons */
        public static final int terminateCommandsOperatorButton = XboxController.Button.kBack.value;
        
    }

    public static final class GeneralConstants {

        public static final boolean fancy_drive = true, 
                                    tFlightHotasOne_drive = true;
        
        public static final double joystick_deadzone = 0.2,
                                   axis_exponent = 1.3;
        
        public static final double starting_yaw = 0; // shoot this might depend based on autonomous... we can make a new command xoxo
        
        public static final double signedPower(double axis_value) {
            axis_value = MathUtil.applyDeadband(axis_value, joystick_deadzone);
            if (axis_value == 0) return 0;
            if (axis_value < 0) return 0 - Math.pow(0 - axis_value, axis_exponent);
            return Math.pow(axis_value, axis_exponent);
        }
        
        /** Normalizes angle to between -180 and 180 */
        public static double normalizeAngle(double degrees) {
            if (degrees < 0) return ((degrees - 180) % 360 + 180);
            return ((degrees + 180) % 360 - 180);
        }

        public static final double getPECorrection(double error, double proportional, double exponent, double min_power, double max_power) {
            if (Math.abs(error) == 0) return 0;

            double multiplier = max_power;
            if (error < 0) {
                error = 0 - error;
                multiplier = -max_power;
            }

            error *= proportional / max_power; // as a proportion of maximum power
            error = Math.min(error, 1);

            double calculated_power = Math.pow(error, exponent) * multiplier;

            return (Math.abs(calculated_power) < min_power) ? 0 : calculated_power;
        }
    }

    public static final class BaseFalconSwerveConstants {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.75); // 28" width -> 22.75" track width
        public static final double wheelBase = Units.inchesToMeters(18.75); // 24" drivetrain length -> 18.75" wheel base
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

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

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder0_zero);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder1_zero);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder2_zero);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TuningConstants.CANCoder3_zero);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = TuningConstants.autonomous_max_linear_speed * 0.3048;
        public static final double kMaxAccelerationMetersPerSecondSquared = TuningConstants.autonomous_max_linear_speed * 0.3048 / TuningConstants.autonomous_ramp_up_time_linear;
        public static final double kMaxAngularSpeedRadiansPerSecond = TuningConstants.autonomous_max_angular_speed * Math.PI / 180.0;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = TuningConstants.autonomous_max_angular_speed * Math.PI / 180.0 / TuningConstants.autonomous_ramp_up_time_angular;
    
        public static final double kPXController = TuningConstants.autonomous_translation_p_controller;
        public static final double kPYController = TuningConstants.autonomous_translation_p_controller;
        public static final double kPThetaController = TuningConstants.autonomous_angle_p_controller;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class TeleopSwerveConstants {
        public static final double swerve_min_pid_rotation = 0.05,
                                   swerve_max_pid_rotation = 0.8, 
                                   swerve_calibration_time = 0.5, 
                                   swerve_min_manual_translation = 0.05, 
                                   swerve_min_manual_rotation = 0.02,

                                   teleop_swerve_slow_factor = 0.5, 
                                   
                                   vision_tolerance = 15, 
                                   
                                   teleop_rotation_percent = 0.75, 
                                   swerve_bumper_turn_sensitivity = 0.35; // ratio of teleop swerve rotation speed vs maximum swerve rotation speed
    }
    
    public static final class IntakeConstants {
        
    }

    public static final class PivotConstants {

    }

    public static final class ShooterConstants {

    }
}

package frc.robot.subsystems;

import frc.lib.util.PIECalculator;
import frc.robot.Constants;
import frc.robot.Variables;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.TeleopSwerveConstants.*;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] swerveModules;
    public Pigeon2 gyro;

    private double last_manual_time;
    private PIECalculator pie;

    public Swerve() {
        pie = new PIECalculator(teleop_angle_p, teleop_angle_i, teleop_angle_e, swerve_min_pid_rotation * Constants.BaseFalconSwerveConstants.maxAngularVelocity, swerve_max_pid_rotation * Constants.BaseFalconSwerveConstants.maxAngularVelocity, starting_yaw);

        gyro = new Pigeon2(Constants.BaseFalconSwerveConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro(starting_yaw);

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.BaseFalconSwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.BaseFalconSwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.BaseFalconSwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.BaseFalconSwerveConstants.Mod3.constants)
        };

        Timer.delay(1.0);
        resetModulesToAbsolute(); // shouldn't be needed

        poseEstimator = new SwerveDrivePoseEstimator(Constants.BaseFalconSwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());

        // PathPlanner

        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds

            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(autonomous_translation_p_controller, 0.0, 0.0), // Translation PID constants
                new PIDConstants(autonomous_angle_p_controller, 0.0, 0.0), // Rotation PID constants
                autonomous_max_linear_speed, // Max module speed, in m/s
                0.5 * Math.sqrt(Constants.BaseFalconSwerveConstants.wheelBase * Constants.BaseFalconSwerveConstants.wheelBase +   // Drive base radius in meters. 
                                Constants.BaseFalconSwerveConstants.trackWidth * Constants.BaseFalconSwerveConstants.trackWidth), // Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                return Variables.side == "red"; // should return true when on red side
            },
            this // Reference to this subsystem to set requirements
        );
    }

    /** Counterclockwise in degrees */
    public void changeHeading(double delta) {
        setTargetHeading(getGyroYaw().getDegrees() + delta);
    }

    public void setTargetHeading(double target) {
        pie.resetTarget(normalizeAngle(target - getGyroYaw().getDegrees()) + getGyroYaw().getDegrees());
        last_manual_time = -999;
    }

    public void brake() {
        changeHeading(0);
        drive(new Translation2d(), 0, true, false);
    }

    public void makeX() {
        changeHeading(0);

        SwerveModuleState[] swerveModuleStates = { // FL, FR, BL, BR
            new SwerveModuleState(0.05, new Rotation2d(Math.PI / 4)), 
            new SwerveModuleState(0.05, new Rotation2d(-Math.PI / 4)), 
            new SwerveModuleState(0.05, new Rotation2d(3.0 * Math.PI / 4)), 
            new SwerveModuleState(0.05, new Rotation2d(-3.0 * Math.PI / 4))
        };
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.BaseFalconSwerveConstants.maxSpeed);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        drive(translation, rotation, fieldRelative, isOpenLoop, true);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean slowDown) {
        
        double current_heading = getGyroYaw().getDegrees();
        
        if (rotation == 0) {
            if (Timer.getFPGATimestamp() - last_manual_time < swerve_calibration_time) {
                pie.resetTarget(current_heading);
            } else {

                if (Math.abs(pie.getTarget() - current_heading) > 180) {
                    pie.resetTarget(current_heading + normalizeAngle(pie.getTarget() - current_heading));
                }
                rotation = pie.update(current_heading);
            }
        } else {
            pie.resetTarget(current_heading);
            last_manual_time = Timer.getFPGATimestamp();
        }

        if (slowDown) {
            translation = translation.times(Variables.slowdown_factor);
            rotation *= (turn_slow_ratio - 1 + Variables.slowdown_factor) / turn_slow_ratio;
        }

        if (translation.getNorm() < swerve_min_manual_translation * Constants.BaseFalconSwerveConstants.maxSpeed) translation = new Translation2d();
        if (Math.abs(rotation) < swerve_min_manual_rotation * Constants.BaseFalconSwerveConstants.maxAngularVelocity) rotation = 0;

        SwerveModuleState[] swerveModuleStates =
            Constants.BaseFalconSwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.BaseFalconSwerveConstants.maxSpeed);

        for(SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by PathPlanner */
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        drive(new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond), chassisSpeeds.omegaRadiansPerSecond, false, true, false);
    }

    /* Used by PathPlanner */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.BaseFalconSwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.BaseFalconSwerveConstants.maxSpeed);
        
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }
    
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void zeroGyro(double yaw) { // resets robot angle. Only do if pigeon is inaccurate or at starting of match
        poseEstimator.resetPosition(Rotation2d.fromDegrees(yaw), getModulePositions(), new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(yaw)));
        gyro.setYaw(yaw);
        pie.resetTarget(yaw);
        last_manual_time = Timer.getFPGATimestamp();
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public double getYawError() {
        return normalizeAngle(pie.getTarget() - getGyroYaw().getDegrees());
    }

    @Override
    public void periodic() {
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());

        Pose2d vision_estimate = Limelight.getRobotPosition();

        if (vision_estimate.getTranslation().getNorm() > 0.1 && (Math.abs(normalizeAngle(getGyroYaw().getDegrees() - vision_estimate.getRotation().getDegrees())) < vision_tolerance)) {
            poseEstimator.addVisionMeasurement(vision_estimate, Timer.getFPGATimestamp() - Limelight.getLatency());
            SmartDashboard.putString("Vision Pose", "(" + Math.round(vision_estimate.getTranslation().getX() * 100) / 100.0 + ", " + Math.round(vision_estimate.getTranslation().getY() * 100) / 100.0 + ")");
            SmartDashboard.putString("Vision Heading", "" + Math.round(vision_estimate.getRotation().getDegrees() * 100) / 100.0 + " degrees");
        } else if (vision_estimate.getTranslation().getNorm() > 0.1) {
            SmartDashboard.putString("Vision Pose", "Vision estimate did not make sense");
            SmartDashboard.putString("Vision Heading", "Vision estimate did not make sense");
        } else {
            SmartDashboard.putString("Vision Pose", "No vision estimate");
            SmartDashboard.putString("Vision Heading", "No vision estimate");
        }

        SmartDashboard.putNumber("Pigeon Yaw", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Pose Estimator Yaw ", getHeading().getDegrees());

        SmartDashboard.putString("Odometry Pose", "(" + Math.round(poseEstimator.getEstimatedPosition().getTranslation().getX() * 100) / 100.0 + ", " + Math.round(poseEstimator.getEstimatedPosition().getTranslation().getY() * 100) / 100.0 + ")");
        SmartDashboard.putString("Odometry Heading", "" + Math.round(poseEstimator.getEstimatedPosition().getRotation().getDegrees() * 100) / 100.0 + " degrees");
        
        // TODO: Make a visualizer in pygame
        
        double angle_current = 0;
        double drive_current = 0;

        for (SwerveModule mod : swerveModules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            angle_current += mod.getCurrents()[0] / 4.0;
            drive_current += mod.getCurrents()[1] / 4.0;
        }

        SmartDashboard.putNumber("Average Angle Motor Current", angle_current);
        SmartDashboard.putNumber("Average Drive Motor Current", drive_current);
        SmartDashboard.putString("Side", Variables.side);
    }

    public void teleopInit() {
        changeHeading(0);
    }
}
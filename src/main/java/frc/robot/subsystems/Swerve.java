package frc.robot.subsystems;

import frc.lib.util.PIECalculator;
import frc.robot.Constants;
import frc.robot.Variables;
import frc.robot.autonomous.ShootingMath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TuningConstants.*;

import java.util.Arrays;

import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.TeleopSwerveConstants.*;

public class Swerve extends SubsystemBase {

    public static SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] swerveModules;
    public Pigeon2 gyro;

    private double last_manual_time;
    private PIECalculator pie;
    StructPublisher<Pose2d> posePublisher;
    StructArrayPublisher<SwerveModuleState> statePublisher;
    StructArrayPublisher<SwerveModuleState> canStatePublisher;
    StructPublisher<Pose2d> visionPublisher;

    public static final int cache_size = 10;

    public static Pose2d[] lastRecordedPoses = new Pose2d[cache_size];
    public static double[] lastRecordedTimes = new double[cache_size];

    public static Translation2d velocity = new Translation2d();

    public Swerve() {
        pie = new PIECalculator(teleop_angle_p, teleop_angle_i, teleop_angle_e, swerve_min_pid_rotation * Constants.BaseFalconSwerveConstants.maxAngularVelocity, swerve_max_pid_rotation * Constants.BaseFalconSwerveConstants.maxAngularVelocity, starting_yaw);

        gyro = new Pigeon2(Constants.BaseFalconSwerveConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(starting_yaw + (Variables.isBlueAlliance ? 0 : 180));

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.BaseFalconSwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.BaseFalconSwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.BaseFalconSwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.BaseFalconSwerveConstants.Mod3.constants)
        };

        Timer.delay(1.0);
        resetModulesToAbsolute(); // shouldn't be needed

        poseEstimator = new SwerveDrivePoseEstimator(Constants.BaseFalconSwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
        last_manual_time = Timer.getFPGATimestamp();

        fillCacheWithPose(poseEstimator.getEstimatedPosition());

        // PathPlanner

        AutoBuilder.configureHolonomic(
            Swerve::getPose, // Robot pose supplier
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
                if (DriverStation.getAlliance().isPresent()) {
                    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                } else {
                    return false;
                }
            },
            this // Reference to this subsystem to set requirements
        );

        posePublisher = NetworkTableInstance.getDefault().getStructTopic("/Swerve/Pose", Pose2d.struct).publish();
        statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Swerve/States", SwerveModuleState.struct).publish();
        canStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Swerve/canStates", SwerveModuleState.struct).publish();
        visionPublisher = NetworkTableInstance.getDefault().getStructTopic("/Swerve/Vision", Pose2d.struct).publish();
    }

    public static void fillCacheWithPose(Pose2d newPose) {
        for (int i = 0; i < cache_size; i++) {
            lastRecordedPoses[i] = new Pose2d(newPose.getX(), newPose.getY(), newPose.getRotation());
            lastRecordedTimes[i] = Timer.getFPGATimestamp();
        }
    }

    public static Translation2d addPoseToCache(Pose2d newPose) {
        Translation2d velocity = newPose.getTranslation().minus(lastRecordedPoses[0].getTranslation()).times(1 / (Timer.getFPGATimestamp() - lastRecordedTimes[0]));

        for (int i = 0; i < cache_size - 1; i++) {
            lastRecordedPoses[i] = new Pose2d(lastRecordedPoses[i + 1].getX(), lastRecordedPoses[i + 1].getY(), lastRecordedPoses[i + 1].getRotation());
            lastRecordedTimes[i] = lastRecordedTimes[i + 1];
        }
        lastRecordedPoses[cache_size - 1] = new Pose2d(newPose.getX(), newPose.getY(), newPose.getRotation());
        lastRecordedTimes[cache_size - 1] = Timer.getFPGATimestamp();

        return velocity;
    }

    /** Counterclockwise in degrees */
    public void changeHeading(double delta) {
        setTargetHeading(getGyroYaw().getDegrees() + delta);
        if (delta == 0) {
            pie.resetTarget(getGyroYaw().getDegrees());
            last_manual_time = Timer.getFPGATimestamp();
        }
    }

    /* Heading must be relative to blue alliance */
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

        if (Variables.invert_rotation) rotation *= -1;

        if (!Variables.isBlueAlliance) translation = translation.times(-1);

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
        drive(new Translation2d(chassisSpeeds.vxMetersPerSecond * (Variables.isBlueAlliance ? 1 : -1), chassisSpeeds.vyMetersPerSecond * (Variables.isBlueAlliance ? 1 : -1)), chassisSpeeds.omegaRadiansPerSecond, false, true, false);
    }

    /** Used by PathPlanner */
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

    public SwerveModuleState[] getCanModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getCanState();
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

    public static Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
        fillCacheWithPose(poseEstimator.getEstimatedPosition());

        gyro.setYaw(pose.getRotation().getDegrees());
        
        brake();
        pie.resetTarget(pose.getRotation().getDegrees());
        last_manual_time = Timer.getFPGATimestamp();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /* Heading must be relative to blue alliance */
    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
        fillCacheWithPose(poseEstimator.getEstimatedPosition());
    }

    /* Heading must be relative to driver */
    public void zeroHeading() {
        poseEstimator.resetPosition(getGyroYaw().plus(Rotation2d.fromDegrees(Variables.isBlueAlliance ? 0 : 180)), getModulePositions(), new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(Variables.isBlueAlliance ? 0 : 180)));
        fillCacheWithPose(poseEstimator.getEstimatedPosition());
    }

    /* Heading must be relative to driver */
    public void zeroGyro(double yaw) { // resets robot angle. Only do if pigeon is inaccurate or at starting of match
        poseEstimator.resetPosition(Rotation2d.fromDegrees(yaw + (Variables.isBlueAlliance ? 0 : 180)), getModulePositions(), new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(yaw + (Variables.isBlueAlliance ? 0 : 180))));
        fillCacheWithPose(poseEstimator.getEstimatedPosition());
        gyro.setYaw(yaw + (Variables.isBlueAlliance ? 0 : 180));

        brake();
        pie.resetTarget(yaw + (Variables.isBlueAlliance ? 0 : 180));
        last_manual_time = Timer.getFPGATimestamp();
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public double getYawError() {
        return normalizeAngle(pie.getTarget() - getGyroYaw().getDegrees());
    }

    public boolean pidCloseEnough() {
        return Math.abs(getYawError()) < 15; // TODO: make this in constants
    }
    
    public void overrideOdometry() {
        resetOdometry(Limelight.getVisionEstimate());
    }

    public void targetSpeaker() {
        setTargetHeading(ShootingMath.drivetrainAngle().getDegrees());
    }

    @Override
    public void periodic() {
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());

        Pose2d vision_estimate = Limelight.getVisionEstimate();

        if (vision_estimate.getTranslation().getNorm() > 0.1 && (Math.abs(normalizeAngle(getGyroYaw().getDegrees() - vision_estimate.getRotation().getDegrees())) < vision_tolerance)) {
            poseEstimator.addVisionMeasurement(vision_estimate, Timer.getFPGATimestamp() - Limelight.getLatency(), VecBuilder.fill(0.8, 0.8, 6 * Math.PI / 180.0));
            log("Vision Pose", "(" + Math.round(vision_estimate.getTranslation().getX() * 100) / 100.0 + ", " + Math.round(vision_estimate.getTranslation().getY() * 100) / 100.0 + ")");
            log("Vision Heading", "" + Math.round(vision_estimate.getRotation().getDegrees() * 100) / 100.0 + " degrees");
        } else if (vision_estimate.getTranslation().getNorm() > 0.1) {
            log("Vision Pose", "Vision estimate did not make sense");
            log("Vision Heading", "Vision estimate did not make sense");
        } else {
            log("Vision Pose", "No vision estimate");
            log("Vision Heading", "No vision estimate");
        }

        velocity = addPoseToCache(poseEstimator.getEstimatedPosition());

        log("Pigeon Yaw", getGyroYaw().getDegrees());
        log("Pose Estimator Yaw ", getHeading().getDegrees());

        log("Odometry Pose", "(" + Math.round(poseEstimator.getEstimatedPosition().getTranslation().getX() * 100) / 100.0 + ", " + Math.round(poseEstimator.getEstimatedPosition().getTranslation().getY() * 100) / 100.0 + ")");
        log("Odometry Heading", "" + Math.round(poseEstimator.getEstimatedPosition().getRotation().getDegrees() * 100) / 100.0 + " degrees");
        
        double angle_current = 0;
        double drive_current = 0;

        for (SwerveModule mod : swerveModules) {
            log("Mod " + mod.moduleNumber + " Cancoder", mod.getCANcoder().getDegrees());
            log("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            log("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            angle_current += mod.getCurrents()[0] / 4.0;
            drive_current += mod.getCurrents()[1] / 4.0;
        }

        log("Average Angle Motor Current", angle_current);
        log("Average Drive Motor Current", drive_current);
        log("Side", Variables.isBlueAlliance ? "Blue" : "Red");
        log("Swerve Close Enough", pidCloseEnough() ? "yes" : "no");

        posePublisher.set(getPose());
        statePublisher.set(getModuleStates());
        canStatePublisher.set(getCanModuleStates());
        visionPublisher.set(Limelight.getVisionEstimate());
    }

    public void teleopInit() {
        changeHeading(0);
    }

  /** Get the position of all drive wheels in radians. */
  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(swerveModules).mapToDouble(SwerveModule::getPositionRads).toArray();
  }
}

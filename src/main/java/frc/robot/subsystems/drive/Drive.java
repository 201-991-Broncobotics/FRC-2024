package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIECalculator;
import frc.robot.Variables;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.Logger;
import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.TeleopSwerveConstants.*;

public class Drive extends SubsystemBase {

  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(22.75);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(18.75);
  private static final double DRIVE_BASE_RADIUS =
    Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  private SwerveDrivePoseEstimator poseEstimator;
  private Module[] modules = new Module[4];
  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

  private double last_manual_time;
  private PIECalculator pie;

  public Drive(GyroIO gyroIO, ModuleIO flModuleIO,
    ModuleIO frModuleIO,
    ModuleIO blModuleIO,
    ModuleIO brModuleIO) {
    pie = new PIECalculator(teleop_angle_p, teleop_angle_i, teleop_angle_e, swerve_min_pid_rotation * MAX_ANGULAR_SPEED, swerve_max_pid_rotation * MAX_ANGULAR_SPEED, 0);

    this.gyroIO = gyroIO;
    gyroIO.setYaw(0);

    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    resetModulesToAbsolute(); // shouldn't be needed

    poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroYaw(), getModulePositions(), new Pose2d());

    // PathPlanner
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::setPose,
      () -> kinematics.toChassisSpeeds(getModuleStates()),
      this::runVelocity,
      new HolonomicPathFollowerConfig(
        MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
      () ->
      DriverStation.getAlliance().isPresent() &&
      DriverStation.getAlliance().get() == Alliance.Red,
      this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
      (activePath) -> {
        Logger.recordOutput(
          "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
      });
    PathPlannerLogging.setLogTargetPoseCallback(
      (targetPose) -> {
        Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
      });
  }

  /** Counterclockwise in degrees */
  public void changeHeading(Rotation2d delta) {
    setTargetHeading(getGyroYaw().plus(delta));
  }

  public void setTargetHeading(Rotation2d target) {
    pie.resetTarget(target.minus(getGyroYaw()).plus(getGyroYaw()).getDegrees());
    last_manual_time = -999;
  }

  public void brake() {
    changeHeading(Rotation2d.fromDegrees(0));
    drive(new Translation2d(), 0, true, false);
  }

  public void makeX() {
    changeHeading(Rotation2d.fromDegrees(0));

    SwerveModuleState[] swerveModuleStates = { // FL, FR, BL, BR
      new SwerveModuleState(0.05, new Rotation2d(Math.PI / 4)),
      new SwerveModuleState(0.05, new Rotation2d(-Math.PI / 4)),
      new SwerveModuleState(0.05, new Rotation2d(3.0 * Math.PI / 4)),
      new SwerveModuleState(0.05, new Rotation2d(-3.0 * Math.PI / 4))
    };

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_LINEAR_SPEED);

    for (Module mod: modules) {
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
          pie.resetTarget(current_heading + Rotation2d.fromDegrees(pie.getTarget() - current_heading).getDegrees());
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

    if (translation.getNorm() < swerve_min_manual_translation * MAX_LINEAR_SPEED) translation = new Translation2d();
    if (Math.abs(rotation) < swerve_min_manual_rotation * MAX_ANGULAR_SPEED) rotation = 0;

    rotation *= -1; // oops

    SwerveModuleState[] swerveModuleStates =
      kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(),
          translation.getY(),
          rotation,
          getHeading()
        ) :
        new ChassisSpeeds(
          translation.getX(),
          translation.getY(),
          rotation)
      );

    Logger.recordOutput("SwerveStates/Setpoints", swerveModuleStates);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_LINEAR_SPEED);

    for (Module mod: modules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by PathPlanner */
  public void runVelocity(ChassisSpeeds chassisSpeeds) {
    drive(new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond), chassisSpeeds.omegaRadiansPerSecond, false, true, false);
  }

  /* Used by PathPlanner */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_LINEAR_SPEED);

    for (Module mod: modules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (Module mod: modules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (Module mod: modules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void resetModulesToAbsolute() {
    for (Module mod: modules) {
      mod.resetToAbsolute();
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
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
    gyroIO.setYaw(yaw);
    pie.resetTarget(yaw);
    last_manual_time = Timer.getFPGATimestamp();
  }

  public Rotation2d getGyroYaw() {
    return gyroInputs.yawPosition;
  }

  public double getYawError() {
    return normalizeAngle(pie.getTarget() - getGyroYaw().getDegrees());
  }

  public double normalizeAngle(double angle) {
    return Rotation2d.fromDegrees(angle).getDegrees();
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module: modules) {
      module.periodic();
    }

    Logger.recordOutput("SwerveStates/Measured", getModuleStates());

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());

    SmartDashboard.putNumber("Pigeon Yaw", getGyroYaw().getDegrees());
    SmartDashboard.putNumber("Pose Estimator Yaw ", getHeading().getDegrees());

    SmartDashboard.putString("Odometry Pose", "(" + Math.round(poseEstimator.getEstimatedPosition().getTranslation().getX() * 100) / 100.0 + ", " + Math.round(poseEstimator.getEstimatedPosition().getTranslation().getY() * 100) / 100.0 + ")");
    SmartDashboard.putString("Odometry Heading", "" + Math.round(poseEstimator.getEstimatedPosition().getRotation().getDegrees() * 100) / 100.0 + " degrees");

    // TODO: Make a visualizer in pygame

    double angle_current = 0;
    double drive_current = 0;

    for (Module mod: modules) {

      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCANCoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      angle_current += mod.getCurrents()[0] / 4.0;
      drive_current += mod.getCurrents()[1] / 4.0;
    }

    SmartDashboard.putNumber("Average Angle Motor Current", angle_current);
    SmartDashboard.putNumber("Average Drive Motor Current", drive_current);
  }

  public void teleopInit() {
    changeHeading(Rotation2d.fromDegrees(0));
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
        new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
        new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
        new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  public void addVisionMeasurement(double xyStds, double degStds, Pose2d pose, double latency) {
    poseEstimator.addVisionMeasurement(pose, latency, VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }
}

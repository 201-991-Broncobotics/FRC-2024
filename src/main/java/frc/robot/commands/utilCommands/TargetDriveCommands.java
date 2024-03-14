package frc.robot.commands.utilCommands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Variables;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.GeneralConstants.normalizeAngle;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class TargetDriveCommands {

    public static Command modifiedPathFindCommand(Pose2d targetPose, double max_distance, double max_angular_distance) {
        return new ConditionalCommand(
          AutoBuilder.pathfindToPose(
              targetPose, 
              new PathConstraints(
                kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared,  
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
              ), 
              0, 
              0
          ), 
          new InstantCommand(), 
          () -> (
              (Math.abs(normalizeAngle(targetPose.getRotation().getDegrees() - Swerve.getPose().getRotation().getDegrees())) < max_angular_distance) &&
              (
                  (Variables.isBlueAlliance && (targetPose.minus(Swerve.getPose()).getTranslation().getNorm() < max_distance)) || 
                  (!Variables.isBlueAlliance && (targetPose.minus(new Pose2d(
                    16.5608 - Swerve.getPose().getX(), Swerve.getPose().getY(), Rotation2d.fromDegrees(180).minus(Swerve.getPose().getRotation())
                  )).getTranslation().getNorm() < max_distance))
              )
          ));
    }

    public static Command driveToAmp(Swerve swerve) {

        // we only want to do this IF we are facing relatively vertical and
        return modifiedPathFindCommand(
            new Pose2d(1.82, 7.41, Rotation2d.fromDegrees(90)),
            4, 
            30
        );
        
    }
    
}
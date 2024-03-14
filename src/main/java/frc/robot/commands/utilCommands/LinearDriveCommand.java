package frc.robot.commands.utilCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.TuningConstants.*;

public class LinearDriveCommand extends Command {
        
    private Swerve swerve;
    private final double power, distance;
    private Pose2d starting_pose;
    private double end_time;

    /** Drive Fieldrelative Forward while doing pid for heading */
    public LinearDriveCommand(Swerve swerve, double distance, double power) {
        this.swerve = swerve;
        this.power = power;
        this.distance = distance;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        starting_pose = Swerve.getPose();
        end_time = Timer.getFPGATimestamp() + 1;
    }

    @Override
    public void execute() {
        // we might do a PID thing...
        double conv_power = Math.min(Math.abs(power), (distance - Swerve.getPose().relativeTo(starting_pose).getTranslation().getNorm()) * teleop_translation_p);
        if (power < 0) conv_power = -conv_power;
        conv_power = power; // we could test without this line soon
        swerve.drive(new Translation2d(conv_power, 0).times(Constants.BaseFalconSwerveConstants.maxSpeed), 0, true, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.brake();
    }
    
    @Override
    public boolean isFinished() {
        return ((Swerve.getPose().relativeTo(starting_pose).getTranslation().getNorm() > distance - teleop_translation_tolerance) || (Timer.getFPGATimestamp() > end_time));
    }
    
}
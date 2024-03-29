package frc.robot.commands.defaultCommands;

import frc.robot.Constants;
import frc.robot.Variables;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.TeleopSwerveConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerveAbsoluteDirecting extends Command {
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier directionXSup;
    private DoubleSupplier directionYSup;
    private DoubleSupplier turnSup;
    private IntSupplier targetSup;
    private BooleanSupplier slowSup;
    private BooleanSupplier forcedDirectingSup;

    public TeleopSwerveAbsoluteDirecting(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier directionXSup, DoubleSupplier directionYSup, IntSupplier targetSup, DoubleSupplier turnSup, BooleanSupplier slowSup, BooleanSupplier forcedDirectingSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.directionXSup = directionXSup;
        this.directionYSup = directionYSup;
        this.targetSup = targetSup;
        this.turnSup = turnSup;
        this.slowSup = slowSup;
        this.forcedDirectingSup = forcedDirectingSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = signedPower(translationSup.getAsDouble());
        double strafeVal = signedPower(strafeSup.getAsDouble());
        double turnVal = signedPower(turnSup.getAsDouble()) * swerve_bumper_turn_sensitivity;

        double slowVal = 1;

        if (slowSup.getAsBoolean()) slowVal = teleop_swerve_slow_factor;
        
        double x_dir = directionXSup.getAsDouble();
        double y_dir = directionYSup.getAsDouble();

        if (x_dir * x_dir + y_dir * y_dir < joystick_deadzone * joystick_deadzone) {
            x_dir = 0;
            y_dir = 0;
        }

        if (turnVal == 0) {
            if (x_dir != 0 || y_dir != 0) {
                double target_heading = swerve.getGyroYaw().getDegrees() - (Variables.isBlueAlliance ? 0 : 180);;

                if (x_dir == 0) {
                    if (y_dir > 0) {
                        target_heading = 0;
                    } else {
                        target_heading = 180;
                    }
                } else if (x_dir < 0) {
                    target_heading = Math.atan(y_dir / x_dir) * 180.0 / Math.PI + 90;
                } else {
                    target_heading = Math.atan(y_dir / x_dir) * 180.0 / Math.PI - 90;
                }
                turnVal = getPECorrection(normalizeAngle(target_heading + (Variables.isBlueAlliance ? 0 : 180) - swerve.getGyroYaw().getDegrees()), teleop_angle_p, teleop_angle_e, swerve_min_pid_rotation * Constants.BaseFalconSwerveConstants.maxAngularVelocity, swerve_max_pid_rotation * Constants.BaseFalconSwerveConstants.maxAngularVelocity);

            } else if (targetSup.getAsInt() % 90 == 0) { // setTargetHeading on purpose
                swerve.setTargetHeading(targetSup.getAsInt() + (Variables.isBlueAlliance ? 0 : 180));
            }
        }

        if (forcedDirectingSup.getAsBoolean()) {
            turnVal = 0;
            swerve.targetSpeaker();
        }

        /* Drive */
        swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.BaseFalconSwerveConstants.maxSpeed).times(slowVal), 
            turnVal * Constants.BaseFalconSwerveConstants.maxAngularVelocity, 
            true, 
            true
        );
    }
}
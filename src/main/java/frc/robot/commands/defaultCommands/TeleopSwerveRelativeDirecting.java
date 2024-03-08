package frc.robot.commands.defaultCommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.TeleopSwerveConstants.*;

public class TeleopSwerveRelativeDirecting extends Command {    
    private Swerve swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private IntSupplier targetSup;
    private DoubleSupplier slowSup;
    private BooleanSupplier forcedDirectingSup;

    public TeleopSwerveRelativeDirecting(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, IntSupplier targetSup, DoubleSupplier slowSup, BooleanSupplier forcedDirectingSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.targetSup = targetSup;
        this.slowSup = slowSup;
        this.forcedDirectingSup = forcedDirectingSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = signedPower(translationSup.getAsDouble());
        double strafeVal = signedPower(strafeSup.getAsDouble());
        double rotationVal = signedPower(rotationSup.getAsDouble()) * teleop_rotation_percent;

        if ((rotationVal) == 0 && (targetSup.getAsInt() % 90 == 0)) swerve.setTargetHeading(targetSup.getAsInt());

        if (forcedDirectingSup.getAsBoolean()) {
            rotationVal = 0;
            swerve.targetSpeaker();
        }

        /* Drive */
        swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.BaseFalconSwerveConstants.maxSpeed).times(slowSup.getAsDouble()), 
            rotationVal * Constants.BaseFalconSwerveConstants.maxAngularVelocity * (turn_slow_ratio - 1 + slowSup.getAsDouble()) / turn_slow_ratio, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
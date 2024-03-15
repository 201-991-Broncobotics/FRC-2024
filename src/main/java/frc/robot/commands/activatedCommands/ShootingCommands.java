package frc.robot.commands.activatedCommands;

import static frc.robot.Constants.TuningConstants.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Variables;
import frc.robot.autonomous.ShootingMath;
import frc.robot.commands.defaultCommands.TeleopPivot;
import frc.robot.commands.defaultCommands.TeleopSwerveRelativeDirecting;
import frc.robot.commands.subcommands.*;
import frc.robot.subsystems.*;

public class ShootingCommands {

    public static Command amp(Pivot pivot) {
        return new SetArmPosition(pivot, amp_angle - pivot_guard_angle);
    }

    public static Command autonomousSpeaker(Swerve swerve, Pivot pivot, Flywheel flywheel, Conveyor conveyor) {
        return new SequentialCommandGroup(

            new InstantCommand(() -> {}, flywheel, conveyor), 
            new InstantCommand(() -> { Variables.bypass_rotation = true; swerve.targetSpeaker(); }), 
            new InstantCommand(() -> { Variables.bypass_angling = true; pivot.setTarget(ShootingMath.pivotAngle().getDegrees()); }),

            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new TeleopSwerveRelativeDirecting(swerve, () -> 0, () -> 0, () -> 0, () -> false, () -> -1, () -> 0.5, () -> true), 
                        Commands.waitUntil(swerve::pidCloseEnough)
                    )
                ), 
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new TeleopPivot(pivot, () -> 0), 
                        Commands.waitUntil(pivot::pidCloseEnough)
                    )
                ), 
                FlywheelCommands.outtake(flywheel)
            ), 

            new InstantCommand(() -> conveyor.outtake()), 
            new WaitCommand(3), 

            new InstantCommand(() -> {
                Variables.bypass_rotation = false;
                Variables.bypass_angling = false;
                conveyor.stop();
                flywheel.stop();
                swerve.brake();
                pivot.brake();
            })
        ).handleInterrupt(() -> {
                Variables.bypass_rotation = false;
                Variables.bypass_angling = false;
                conveyor.stop();
                flywheel.stop();
                swerve.brake();
                pivot.brake();
            }
        );
    }

    public static Command speaker(Pivot pivot, Flywheel flywheel) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {}, flywheel), 
            new InstantCommand(() -> { Variables.bypass_angling = true; }),
            FlywheelCommands.outtake(flywheel)
        ).handleInterrupt(() -> {
                flywheel.stop();
                Variables.bypass_angling = false;
            }
        );
    }
}

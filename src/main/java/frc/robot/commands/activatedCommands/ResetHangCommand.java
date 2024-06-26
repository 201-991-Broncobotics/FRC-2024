package frc.robot.commands.activatedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Hang;

public class ResetHangCommand extends SequentialCommandGroup { // basically, run the motors until they run up in amperage

    public ResetHangCommand(Hang hang) {
        super(
            new InstantCommand(() -> { hang.disableLimiting(); hang.moveVoltagePercent(0.15); hang.setCoastModes(); }, hang), 
            new WaitCommand(0.15), 
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    Commands.waitUntil(() -> hang.isLeftStuck()), 
                    new InstantCommand(() -> hang.moveLeftVoltagePercent(0)), 
                    new WaitCommand(0.05), 
                    new InstantCommand(() -> hang.moveLeftVoltagePercent(-0.25)), 
                    new WaitCommand(0.2), 
                    new InstantCommand(() -> hang.moveLeftVoltagePercent(0))
                ), new SequentialCommandGroup(
                    Commands.waitUntil(() -> hang.isRightStuck()), 
                    new InstantCommand(() -> hang.moveRightVoltagePercent(0)), 
                    new WaitCommand(0.05), 
                    new InstantCommand(() -> hang.moveRightVoltagePercent(-0.25)), 
                    new WaitCommand(0.2), 
                    new InstantCommand(() -> hang.moveRightVoltagePercent(0))
                )
            ), 
            new InstantCommand(() -> { hang.setBrakeModes(); hang.resetEncoders(); hang.enableLimiting(); })
        );
    }

}

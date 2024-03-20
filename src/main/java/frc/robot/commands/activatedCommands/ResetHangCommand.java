package frc.robot.commands.activatedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Hang;

public class ResetHangCommand extends SequentialCommandGroup { // basically, run the motors until they run up in amperage

    public ResetHangCommand(Hang hang) {
        super(
            new InstantCommand(() -> { hang.moveVoltage(0.15); hang.setCoastModes(); }, hang), 
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.1), 
                    Commands.waitUntil(() -> hang.isLeftStuck()), 
                    new InstantCommand(() -> hang.moveLeftVoltage(0)), 
                    new WaitCommand(0.05), 
                    new InstantCommand(() -> hang.moveLeftVoltage(-0.3)), 
                    new WaitCommand(0.4), 
                    new InstantCommand(() -> hang.moveLeftVoltage(0))
                ), new SequentialCommandGroup(
                    new WaitCommand(0.1), 
                    Commands.waitUntil(() -> hang.isRightStuck()), 
                    new InstantCommand(() -> hang.moveRightVoltage(0)), 
                    new WaitCommand(0.05), 
                    new InstantCommand(() -> hang.moveRightVoltage(-0.3)), 
                    new WaitCommand(0.4), 
                    new InstantCommand(() -> hang.moveRightVoltage(0))
                )
            ), 
            new InstantCommand(() -> { hang.setBrakeModes(); hang.resetEncoders(); hang.enableLimiting(); })
        );
    }

}

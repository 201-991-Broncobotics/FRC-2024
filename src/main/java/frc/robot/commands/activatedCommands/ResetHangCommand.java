package frc.robot.commands.activatedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Hang;

public class ResetHangCommand extends SequentialCommandGroup { // basically, run the motors until they run up in amperage

    public ResetHangCommand(Hang hang) {
        super(
            new InstantCommand(() -> hang.move(0.32), hang), 
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.5), 
                    Commands.waitUntil(() -> hang.isLeftBusy()), 
                    new InstantCommand(() -> hang.moveLeft(0)), 
                    new WaitCommand(0.2), 
                    new InstantCommand(() -> hang.moveLeft(-0.3)), 
                    new WaitCommand(0.4), 
                    new InstantCommand(() -> hang.moveLeft(0))
                ), new SequentialCommandGroup(
                    new WaitCommand(0.5), 
                    Commands.waitUntil(() -> hang.isRightBusy()), 
                    new InstantCommand(() -> hang.moveRight(0)), 
                    new WaitCommand(0.2), 
                    new InstantCommand(() -> hang.moveRight(-0.3)), 
                    new WaitCommand(0.4), 
                    new InstantCommand(() -> hang.moveRight(0))
                )
            ), 
            new InstantCommand(() -> { hang.resetEncoders(); hang.enableLimiting(); })
        );
    }

}

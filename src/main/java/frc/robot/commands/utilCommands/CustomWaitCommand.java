package frc.robot.commands.utilCommands;

import edu.wpi.first.wpilibj2.command.*;

public class CustomWaitCommand extends WaitCommand {

    private final Runnable interruptFunction;

    public CustomWaitCommand(double time, Runnable interruptFunction) {
        super(time);

        this.interruptFunction = interruptFunction;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) interruptFunction.run();
    }

}
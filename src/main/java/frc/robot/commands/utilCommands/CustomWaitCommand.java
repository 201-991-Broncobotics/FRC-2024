package frc.robot.commands.utilCommands;

import edu.wpi.first.wpilibj2.command.*;

public class CustomWaitCommand extends WaitCommand {

    private final Runnable function;

    public CustomWaitCommand(double time, Runnable function) {
        super(time);

        this.function = function;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        function.run();
    }

}
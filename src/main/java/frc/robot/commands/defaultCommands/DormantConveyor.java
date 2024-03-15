package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Conveyor;

public class DormantConveyor extends Command {

    // private Conveyor conveyor;

    public DormantConveyor(Conveyor conveyor) {
        // this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        // conveyor.stop();
    }
    
}
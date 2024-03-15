package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Flywheel;

public class DormantFlywheel extends Command {

    // private Flywheel flywheels;

    public DormantFlywheel(Flywheel flywheels) {
        // this.flywheels = flywheels;
        addRequirements(flywheels);
    }

    @Override
    public void execute() {
        // flywheels.stop();
    }
    
}
package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class DormantIntake extends Command {

    private Intake intake;

    public DormantIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.stop();
    }
    
}
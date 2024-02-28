package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.TuningConstants.*;

public class IntakeCommand extends Command {
    private Intake intake;
    private Conveyor conveyor;

    private double starting_time = 0;
    private double finish_time = 0;

    public IntakeCommand(Intake intake, Conveyor conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;

        addRequirements(intake, conveyor);
    }

    @Override
    public void initialize() {
        intake.intake();
        conveyor.intake();

        starting_time = Timer.getFPGATimestamp();
        finish_time = Timer.getFPGATimestamp() + add_intake_time;
    }

    @Override
    public void execute() {
        if (!intake.isFree() || Timer.getFPGATimestamp() < starting_time + min_intake_time) {
            finish_time = Timer.getFPGATimestamp() + add_intake_time;
        }
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > finish_time;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stop();
    }
}

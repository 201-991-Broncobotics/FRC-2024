package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;

import static frc.robot.Constants.TuningConstants.*;

public class Amp_Subcommand extends Command {
    private Flywheel flywheel;
    private Conveyor conveyor;

    private double starting_time = 0, finish_time = 0;
    
    private boolean ready_to_outtake = false;

    public Amp_Subcommand(Flywheel flywheel, Conveyor conveyor) {
        this.flywheel = flywheel;
        this.conveyor = conveyor;

        addRequirements(flywheel, conveyor);
    }

    @Override
    public void initialize() {
        flywheel.amp(); // in theory we should wait for like 2-3 seconds
        conveyor.stop();

        starting_time = Timer.getFPGATimestamp();
        finish_time = starting_time + min_flywheel_acceleration_time + min_outtake_time;

        ready_to_outtake = false;
    }

    @Override
    public void execute() {
        if (!ready_to_outtake && Timer.getFPGATimestamp() > starting_time + min_flywheel_acceleration_time) {
            conveyor.amp();
        }
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > finish_time;
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        conveyor.stop();
    }
}

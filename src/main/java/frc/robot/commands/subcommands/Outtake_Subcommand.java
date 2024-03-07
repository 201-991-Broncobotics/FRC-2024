package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;

import static frc.robot.Constants.TuningConstants.*;

public class Outtake_Subcommand extends Command {
    private Flywheel flywheel;
    private Conveyor conveyor;

    private double starting_time = 0;
    
    private boolean ready_to_outtake = false;

    public Outtake_Subcommand(Flywheel flywheel, Conveyor conveyor) {
        this.flywheel = flywheel;
        this.conveyor = conveyor;

        addRequirements(flywheel, conveyor);
    }

    @Override
    public void initialize() {

        if (SmartDashboard.getNumber("Current Pivot Angle", 0) < min_outtake_angle) {
            this.cancel();
            return;
        }

        flywheel.outtake(); // in theory we should wait for like 2-3 seconds
        conveyor.stop();

        starting_time = Timer.getFPGATimestamp();

        ready_to_outtake = false;
    }

    @Override
    public void execute() {
        if (!ready_to_outtake && /* flywheel.isFree() && */ Timer.getFPGATimestamp() > starting_time + min_flywheel_acceleration_time) {
            ready_to_outtake = true;
            conveyor.outtake();
            starting_time = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        return ready_to_outtake && flywheel.isFree() && /* conveyor.isFree() && */ Timer.getFPGATimestamp() > starting_time + min_outtake_time;
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        conveyor.stop();
    }
}

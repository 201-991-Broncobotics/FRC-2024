package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.subsystems.Swerve;

public class Autonomous {

    public static Command getAutonomousCommand(Swerve swerve) { // RIP Legacy Code </3

        // String selectedAuto = SmartDashboard.getString("Auto Selector String", "NotAmpTriple");
        //
        // SmartDashboard.putString("Autonomous", (Variables.isBlueAlliance ? "Blue" : "Red") + " " + selectedAuto);
        
        var selectedAuto = "Center 3C0F";
        return new PathPlannerAuto(selectedAuto);
    }
}

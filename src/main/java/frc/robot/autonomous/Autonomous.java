package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class Autonomous {

    public static Command getAutonomousCommand(Swerve swerve) { // RIP Legacy Code </3

        String allianceString = "Blue", location = "Middle", numElements = "";

        double[] temp = SmartDashboard.getNumberArray("Auto Data", new double[] {-1});
        if (temp.length != 1) {
            System.out.println((int) temp[0]);
            switch ((int) temp[0]) {
                case 0:
                    allianceString = "Blue";
                    Limelight.setSide("blue");
                    break;
                case 1:
                    allianceString = "Red";
                    Limelight.setSide("red");
                    break;
            }
            switch ((int) temp[1]) {
                case 0:
                    location = "Amp";
                    break;
                case 1:
                    location = "Middle";
                    break;
                case 2:
                    location = "NotAmp";
                    break;
            }
            switch ((int) temp[2]) {
                case 0:
                    numElements = "";
                    break;
                case 1:
                    numElements = "Double";
                    break;
                case 2:
                    numElements = "Triple";
                    break;
            }
        }

        String selectedAuto = /* allianceString + */ location + numElements;
            // form of commands: [Blue/Red] [Amp/Middle/NotAmp] [/Double/Triple]
            // sadly, LimeLight removed a lot of fun from this

        SmartDashboard.putString("Autonomous", allianceString + " " + selectedAuto);

        return new PathPlannerAuto(selectedAuto);
    }
}
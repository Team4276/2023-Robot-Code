package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.modes.LSMobility;
import frc.robot.auto.modes.LSPickup1;
import frc.robot.auto.modes.MobilityBalance;
import frc.robot.auto.modes.OUTTAKE;
import frc.robot.auto.modes.SSMobility;
import frc.robot.auto.modes.ScoreHigh;

public class AutoPicker {
    SendableChooser<Command> chooser = new SendableChooser<Command>();

    public AutoPicker(){

        chooser.setDefaultOption("Do nothing", null);
        chooser.addOption("Mobility Balance", new MobilityBalance());
        chooser.addOption("SSMobility", new SSMobility());
        chooser.addOption("LSMobility", new LSMobility());
        chooser.addOption("OUTTAKE", new OUTTAKE());

        SmartDashboard.putData("Autos: ", chooser);

    }

    public Command getAutoCommand(){
        return chooser.getSelected();
    }



    //TODO: Rework the paths to be segmented like 254
    //TODO: look into auto architecture
    //TODO: give heading control to the code

   


}
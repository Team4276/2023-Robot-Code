package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.commands.ScoreHighCommand;
import frc.robot.auto.commands.ScoreLowBackCommand;
import frc.robot.auto.commands.ScoreMidCommand;
import frc.robot.auto.modes.LSMobility;
import frc.robot.auto.modes.LSScore2Bal;
import frc.robot.auto.modes.LSScore3;
import frc.robot.auto.modes.MobilityBalance;
import frc.robot.auto.modes.SSMobility;
import frc.robot.auto.modes.SSScore2Bal;
import frc.robot.auto.modes.SSScore3;

public class AutoPicker {
    SendableChooser<Command> chooser;

    public followPathWithEvents followPathWithEvents = new followPathWithEvents();

    public AutoPicker(){
        chooser = new SendableChooser<Command>();

        chooser.setDefaultOption("Do nothing", null);
        chooser.addOption("Mobility Balance", new MobilityBalance());
        chooser.addOption("ScoreHighCommand", new ScoreHighCommand());
        chooser.addOption("ScoreMidCommand", new ScoreMidCommand());
        chooser.addOption("ScoreLowBackCommand", new ScoreLowBackCommand());
        chooser.addOption("LSMobility", new LSMobility());
        chooser.addOption("LSScore2Bal", new LSScore2Bal());
        chooser.addOption("LSScore3", new LSScore3());
        chooser.addOption("SSMobility", new SSMobility());
        chooser.addOption("SSScore2Bal", new SSScore2Bal());
        chooser.addOption("SSScore3", new SSScore3());

        SmartDashboard.putData("Aut: ", chooser);

    }

    public Command getAutoCommand(){
        return chooser.getSelected();
    }


    //TODO: look into auto architecture
    //TODO: give heading control to the code

   


}
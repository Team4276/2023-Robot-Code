package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.modes.MobilityBalance;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NewElbow;

public class AutoPicker {
    SendableChooser<Command> chooser = new SendableChooser<Command>();
    
    DriveSubsystem driveSubsystem;
    NewElbow newElbow;
    Intake intake;

    public AutoPicker(
        DriveSubsystem driveSubsystem,
        NewElbow newElbow,
        Intake intake){
        this.driveSubsystem = driveSubsystem;
        this.newElbow = newElbow;
        this.intake = intake;

        chooser.setDefaultOption("Do nothing", null);
        chooser.addOption("Mobility Balance", new MobilityBalance(driveSubsystem));

        SmartDashboard.putData("Auto Test: ", chooser);

    }

    public Command getAutoCommand(){
        return chooser.getSelected();
    }



    //TODO: Rework the paths to be segmented like 254
    //TODO: look into auto architecture
    //TODO: give heading control to the code

   


}
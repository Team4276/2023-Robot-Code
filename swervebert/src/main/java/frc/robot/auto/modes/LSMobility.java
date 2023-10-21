package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.followPathWithEvents;
import frc.robot.auto.commands.ScoreHighCommand;
import frc.robot.subsystems.DriveSubsystem;

public class LSMobility extends SequentialCommandGroup {
    private DriveSubsystem mDriveSubsystem;

    private String path = "LSMobility";

    public LSMobility(){
        mDriveSubsystem = DriveSubsystem.getInstance();

        addCommands(   
        new ScoreHighCommand(),
        followPathWithEvents.followPPPEvents(path, mDriveSubsystem, 3.5));
    }
    
}

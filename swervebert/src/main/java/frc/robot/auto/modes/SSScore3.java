package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.followPathWithEvents;
import frc.robot.auto.commands.ScoreHighCommand;
import frc.robot.auto.commands.ScoreLowBackCommand;
import frc.robot.auto.commands.ScoreMidCommand;
import frc.robot.subsystems.DriveSubsystem;

public class SSScore3 extends SequentialCommandGroup {
    private DriveSubsystem mDriveSubsystem;

    private String path1 = "SSScorePickup1";
    private String path2 = "SSScorePickup2";

    public SSScore3(){
        mDriveSubsystem = DriveSubsystem.getInstance();

        addCommands(
            new ScoreHighCommand(),
            followPathWithEvents.followPPPEvents(path1, mDriveSubsystem, 3.5),
            new ScoreMidCommand(),
            followPathWithEvents.followPPPEvents(path2, mDriveSubsystem, 3.5)



            

        );
    }

}

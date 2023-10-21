package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.followPathWithEvents;
import frc.robot.auto.commands.OldBalance;
import frc.robot.auto.commands.ScoreHighCommand;
import frc.robot.auto.commands.ScoreMidCommand;
import frc.robot.subsystems.DriveSubsystem;

public class SSScore2Bal extends SequentialCommandGroup {
    private DriveSubsystem mDriveSubsystem;
    private OldBalance oldBalance;

    private String path1 = "SSScorePickup1";
    private String path2 = "SSSCoreBal";

    public SSScore2Bal(){
        mDriveSubsystem = DriveSubsystem.getInstance();

        addCommands(
            new ScoreHighCommand(),
            followPathWithEvents.followPPPEvents(path1, mDriveSubsystem, 3.5),
            new ScoreMidCommand(),
            followPathWithEvents.followPPPEvents(path2, mDriveSubsystem, 3.5),
            new RunCommand(() -> oldBalance.balance(), mDriveSubsystem)



            

        );
    }

}

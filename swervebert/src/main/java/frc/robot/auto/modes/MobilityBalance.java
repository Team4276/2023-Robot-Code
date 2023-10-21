package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.followPathWithEvents;
import frc.robot.auto.commands.OldBalance;
import frc.robot.auto.commands.ScoreHighCommand;
import frc.robot.subsystems.DriveSubsystem;

public class MobilityBalance extends SequentialCommandGroup{
    private DriveSubsystem mDriveSubsystem;

    private String path = "Balance";

    public MobilityBalance(){
        mDriveSubsystem = DriveSubsystem.getInstance();

        OldBalance oldBalance = new OldBalance();

        addRequirements(mDriveSubsystem);

        //TODO: reverse the start positions of autos
        addCommands(
            new ScoreHighCommand(),
            followPathWithEvents.followPPPEvents(path, mDriveSubsystem,1),
            new RunCommand(() -> oldBalance.balance())

        );
    }

}
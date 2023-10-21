package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auto.followPathWithEvents;
import frc.robot.auto.commands.OldBalance;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NewElbow;

public class MobilityBalance extends SequentialCommandGroup{
    private DriveSubsystem mDriveSubsystem;
    private Intake mIntake;
    private final NewElbow mNewElbow;


    private String path = "Balance";

    
    public followPathWithEvents followPathWithEvents = new followPathWithEvents();

    public MobilityBalance(){
        mDriveSubsystem = DriveSubsystem.getInstance();
        mIntake= Intake.getInstance();
        mNewElbow = NewElbow.getInstance();

        OldBalance oldBalance = new OldBalance();

        addRequirements(mDriveSubsystem, mIntake);

        //TODO: reverse the start positions of autos
        addCommands(
            new InstantCommand(() -> mIntake.idle()),
            mNewElbow.ScoreHigh(),
            new WaitUntilCommand(() -> mNewElbow.isStable()),
            new ParallelCommandGroup(
                new InstantCommand(() -> mIntake.outtake()),
                new WaitCommand(0.4)
            ),
            new InstantCommand(() -> mIntake.idle()),
            mNewElbow.Stow(),
            new WaitUntilCommand(() -> mNewElbow.isStable())
,
            followPathWithEvents.followPPPEvents(path, mDriveSubsystem,1.2),
            new RunCommand(() -> oldBalance.balance())

        );
    }

}
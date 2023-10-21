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

public class LSScore2Bal extends SequentialCommandGroup {
    private OldBalance oldBalance;
    private DriveSubsystem mDriveSubsystem;
    private Intake mIntake;
    private final NewElbow mNewElbow;

    private String path1 = "LSScorePickup1";
    private String path2 = "LSScore2Bal";

    
    public followPathWithEvents followPathWithEvents = new followPathWithEvents();

    public LSScore2Bal(){
        mDriveSubsystem = DriveSubsystem.getInstance();
        mIntake = Intake.getInstance();
        mNewElbow = NewElbow.getInstance();

        
        addRequirements(mIntake,mDriveSubsystem);

        addCommands(
            new InstantCommand(() -> mIntake.idle()),
            mNewElbow.ScoreHigh(),
            new WaitUntilCommand(() -> mNewElbow.isStable()),
            new ParallelCommandGroup(
                new InstantCommand(() -> mIntake.outtake()),
                new WaitCommand(0.2)
            ),
            new InstantCommand(() -> mIntake.idle()),
            mNewElbow.Stow(),
            new WaitUntilCommand(() -> mNewElbow.isStable()),
            followPathWithEvents.followPPPEvents(path1, mDriveSubsystem, 3.5),
            new InstantCommand(() -> mIntake.idle()),
            mNewElbow.ScoreMid(),
            new WaitUntilCommand(() -> mNewElbow.isStable()),
            new ParallelCommandGroup(
                new InstantCommand(() -> mIntake.outtake()),
                new WaitCommand(0.2)
            ),
            new InstantCommand(() -> mIntake.idle()),
            mNewElbow.Stow(),
            new WaitUntilCommand(() -> mNewElbow.isStable()),
            followPathWithEvents.followPPPEvents(path2, mDriveSubsystem, 3.5),
            new RunCommand(() -> oldBalance.balance(), mDriveSubsystem)



            

        );
    }

}

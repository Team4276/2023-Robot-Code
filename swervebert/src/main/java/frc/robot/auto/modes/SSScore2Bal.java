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

public class SSScore2Bal extends SequentialCommandGroup {
    private DriveSubsystem mDriveSubsystem;
    private Intake mIntake;
    private final NewElbow mNewElbow;
    private OldBalance oldBalance;

    private String path1 = "SSScorePickup1";
    private String path2 = "SSScore2Bal";

    private final double maxSpeed = 2.5;

    
    public followPathWithEvents followPathWithEvents = new followPathWithEvents();

    public SSScore2Bal(){
        mDriveSubsystem = DriveSubsystem.getInstance();
        mIntake = Intake.getInstance();
        mNewElbow = NewElbow.getInstance();

        
        addRequirements(mIntake, mDriveSubsystem);

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
            followPathWithEvents.followPPPEvents(path1, mDriveSubsystem, maxSpeed),
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
            followPathWithEvents.followPPPEvents(path2, mDriveSubsystem, maxSpeed),
            new RunCommand(() -> oldBalance.balance(), mDriveSubsystem)



            

        );
    }

}

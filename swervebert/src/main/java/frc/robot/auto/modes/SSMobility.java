package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auto.followPathWithEvents;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NewElbow;

public class SSMobility extends SequentialCommandGroup {
    private DriveSubsystem mDriveSubsystem;
    private Intake mIntake;
    private final NewElbow mNewElbow;

    private String path = "SSMobility";

    
    public followPathWithEvents followPathWithEvents = new followPathWithEvents();

    public SSMobility(){
        mDriveSubsystem = DriveSubsystem.getInstance();
        mIntake = Intake.getInstance();
        mNewElbow = NewElbow.getInstance();

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
        followPathWithEvents.followPPPEvents(path, mDriveSubsystem, 3.5));
    }
    
}

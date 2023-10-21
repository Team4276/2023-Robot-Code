package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.followPathWithEvents;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NewElbow;

public class SSMobility extends SequentialCommandGroup {
    private DriveSubsystem mDriveSubsystem;
    private NewElbow mNewElbow;
    private Intake mIntake;

    private followPathWithEvents followPathWithEvents = new followPathWithEvents();

    private String path = "Mobility";

    public SSMobility(){
        mDriveSubsystem = DriveSubsystem.getInstance();
        mNewElbow = NewElbow.getInstance();
        mIntake = Intake.getInstance();

        addCommands(
            mNewElbow.ScoreHigh(),
            new WaitCommand(1),
            new ParallelCommandGroup(
                new RunCommand(() -> mIntake.outtake()),
                new WaitCommand(0.2)),
            mNewElbow.Stow(),
            followPathWithEvents.followPPPEvents(path, mDriveSubsystem, 3.5)
            );
    }
    
}

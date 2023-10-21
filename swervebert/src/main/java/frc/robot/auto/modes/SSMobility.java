package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    private String path = "SSMobility";

    public SSMobility(){
        mDriveSubsystem = DriveSubsystem.getInstance();
        mNewElbow = NewElbow.getInstance();
        mIntake = Intake.getInstance();

        
        addRequirements(mIntake);

        addCommands(   
            new ParallelCommandGroup(
            new InstantCommand(() -> mIntake.outtake()),
            new WaitCommand(0.2)
        ),
        new InstantCommand(() -> mIntake.idle()),
            followPathWithEvents.followPPPEvents(path, mDriveSubsystem, 3.5)
            );
    }
    
}

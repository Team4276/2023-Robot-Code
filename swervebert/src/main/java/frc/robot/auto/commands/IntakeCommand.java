package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NewElbow;

public class IntakeCommand extends SequentialCommandGroup {
    private final Intake mIntake = Intake.getInstance();
    private final NewElbow mNewElbow = NewElbow.getInstance();

    public IntakeCommand(){
        addRequirements(mIntake);

        addCommands(
            new InstantCommand(() -> mIntake.idle()),
            mNewElbow.Intake(),
            new ParallelCommandGroup(
                new InstantCommand(() -> mIntake.intake()),
                new WaitCommand(1.5)
            ),
            new InstantCommand(() -> mIntake.idle()),
            mNewElbow.Stow()
        );
    }
    
}

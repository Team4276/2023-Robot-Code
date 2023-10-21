package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class ScoreLowBackCommand extends SequentialCommandGroup {
    private final Intake mIntake = Intake.getInstance();

    public ScoreLowBackCommand(){
        addRequirements(mIntake);

        addCommands(
            new ParallelCommandGroup(
                new InstantCommand(() -> mIntake.outtake()),
                new WaitCommand(0.2)
            ),
            new InstantCommand(() -> mIntake.idle())
            
        );
    }
    
}

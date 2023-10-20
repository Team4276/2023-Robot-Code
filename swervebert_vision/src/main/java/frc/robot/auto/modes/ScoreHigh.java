package frc.robot.auto.modes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NewElbow;
import pabeles.concurrency.IntOperatorTask.Min;

public class ScoreHigh extends SequentialCommandGroup {
    private NewElbow mNewElbow;
    private Intake mIntake;

    public ScoreHigh(){
        mNewElbow = NewElbow.getInstance();
        mIntake = Intake.getInstance();

        addRequirements(mIntake);

        addCommands(
            new InstantCommand(() -> mIntake.idle()),
            mNewElbow.ScoreHigh(),
            new WaitCommand(3),
            new ParallelCommandGroup(
                new InstantCommand(() -> mIntake.outtake()),
                new WaitCommand(0.2)),
            new InstantCommand(() -> mIntake.idle()),
            mNewElbow.Stow()
        );
    }
    
}

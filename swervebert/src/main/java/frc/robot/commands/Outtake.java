package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class Outtake extends ParallelCommandGroup {
    IntakeSubsystem intakeSubsystem;

    public Outtake(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);

        addCommands(
            intakeSubsystem.out(),
            new WaitCommand(0.2)
        );
        
    }
    
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmSubsystemConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.target;

public class IntakeConeDouble extends ParallelCommandGroup {
    public IntakeConeDouble(
        ArmSubsystem armSubsystem, 
        IntakeSubsystem intakeSubsystem, 
        DriveSubsystem driveSubsystem, 
        LimeLightSubsystem limelightSubsystem,
        boolean isTeleop){

        addRequirements(
            armSubsystem,
            intakeSubsystem,
            driveSubsystem
        );

        if(isTeleop){
            addCommands(
                driveSubsystem.followPathCommand(limelightSubsystem.aligne(target.INTAKE_CONE_DOUBLE)),
                armSubsystem.set(ArmSubsystemConstants.intakeConeDouble),  
                new SequentialCommandGroup(    
                    new ParallelCommandGroup(
                        new WaitUntilCommand(limelightSubsystem::checkIfAligned),
                        new WaitUntilCommand(armSubsystem::isStable)),
                    new InstantCommand(() -> {intakeSubsystem.in();})
                )
            );
        } else {
            addCommands(
                armSubsystem.set(ArmSubsystemConstants.intakeConeDouble),
                new SequentialCommandGroup(        
                    new WaitUntilCommand(armSubsystem::isStable),
                    new InstantCommand(() -> {intakeSubsystem.in();})
                )
            );
        }
    }
}
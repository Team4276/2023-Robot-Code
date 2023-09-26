package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
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

public class ScoreConeHigh extends ParallelCommandGroup {
    public ScoreConeHigh(
        ArmSubsystem armSubsystem, 
        IntakeSubsystem intakeSubsystem, 
        DriveSubsystem driveSubsystem, 
        LimeLightSubsystem limelightSubsystem,
        XboxController xboxController,
        boolean isTeleop){

        addRequirements(
            armSubsystem,
            intakeSubsystem,
            driveSubsystem
        );



        if(isTeleop){
            addCommands(
                driveSubsystem.followPathCommand(limelightSubsystem.aligne(target.SCORE_CONE_HIGH)),
                armSubsystem.set(ArmSubsystemConstants.scoreConeHigh),  
                new SequentialCommandGroup(    
                    new ParallelCommandGroup(
                        new WaitUntilCommand(limelightSubsystem::checkIfAligned),
                        new WaitUntilCommand(armSubsystem::isStable)),
                    new WaitUntilCommand(xboxController::getAButton),
                    new InstantCommand(() -> {intakeSubsystem.out();})
                )
            );
        } else {
            addCommands(
                armSubsystem.set(ArmSubsystemConstants.scoreConeHigh), 
                new SequentialCommandGroup(        
                    new WaitUntilCommand(armSubsystem::isStable),
                    new InstantCommand(() -> {intakeSubsystem.out();})
                )
            );
        }




    }

    
}

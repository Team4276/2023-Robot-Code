package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DriveSubsystem;


public class SpinnyWEWEE extends ParallelCommandGroup {
    DriveSubsystem driveSubsystem;


    private double startDeg = 0;

    // Will spin for about 4 seconds (one rotation)
    public SpinnyWEWEE(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;


        addRequirements(driveSubsystem);

        addCommands(
            new InstantCommand(() -> init()),
            new RunCommand(() -> driveSubsystem.drive(0, 0, 0.5, false, true), driveSubsystem),
            new SequentialCommandGroup(            
                new WaitCommand(1),
                new WaitUntilCommand(() -> checkAngle())
            )
        );
    }

    private void init(){
        startDeg = driveSubsystem.getHeading();


    }

    private boolean checkAngle(){
        double diff = 0;

        if (startDeg > 0){
            if (driveSubsystem.getHeading() > 0){
                diff = startDeg - driveSubsystem.getHeading();
            } else if (driveSubsystem.getHeading() < 0) {
                diff = startDeg - (360 + driveSubsystem.getHeading());
            } else {
                diff = startDeg;
            }
        } else if (startDeg < 0){
            if (driveSubsystem.getHeading() < 0){
                diff = startDeg - driveSubsystem.getHeading();
            } else if (driveSubsystem.getHeading() > 0) {
                diff = startDeg - (driveSubsystem.getHeading() - 360);
            } else {
                diff = startDeg;
            }

        } else {
            diff = driveSubsystem.getHeading();

        }


        if (Math.abs(diff) > 3){
            return false;

        } else {
            return true;

        }
    }
}

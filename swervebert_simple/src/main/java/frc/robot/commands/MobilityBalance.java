package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class MobilityBalance extends SequentialCommandGroup{

    public MobilityBalance(DriveSubsystem driveSubsystem){
        addRequirements(driveSubsystem);

        addCommands(
            new ParallelCommandGroup(
                new RunCommand(
                    () -> driveSubsystem.drive(0.2, 0, 0, false, true),
                    driveSubsystem),
                new WaitCommand(3)
            ),
            new RunCommand(() -> new Balance(driveSubsystem.getPitch()), driveSubsystem)


        );
    }

}
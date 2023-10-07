package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class MobilityBalance extends SequentialCommandGroup{

    public MobilityBalance(DriveSubsystem driveSubsystem){
        addRequirements(driveSubsystem);

        OldBalance oldBalance = new OldBalance(driveSubsystem);

        //TODO: test direction for non field relative commands calibrate the speed over charge station-p

        addCommands(
            new ParallelCommandGroup(
                new RunCommand(
                    () -> driveSubsystem.drive(0.2, 0, 0, false, false),
                    driveSubsystem),
                new WaitCommand(3)
            ),
            new ParallelCommandGroup(
                new RunCommand(
                    () -> driveSubsystem.drive(-0.2, 0, 0, false, false),
                    driveSubsystem),
                new WaitCommand(1)
            ),
            new RunCommand(() -> oldBalance.balance())

        );
    }

}
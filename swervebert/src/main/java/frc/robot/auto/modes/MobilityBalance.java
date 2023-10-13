package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.OldBalance;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NewElbow;

public class MobilityBalance extends SequentialCommandGroup{
    private DriveSubsystem mDriveSubsystem;
    private NewElbow mNewElbow;
    private Intake mIntake;

    public MobilityBalance(){
        mDriveSubsystem = DriveSubsystem.getInstance();
        mNewElbow = NewElbow.getInstance();
        mIntake = Intake.getInstance();

        OldBalance oldBalance = new OldBalance(mDriveSubsystem);

        //TODO: test direction for non field relative commands calibrate the speed over charge station-p

        addCommands(
            mNewElbow.ScoreHigh(),
            new WaitCommand(1),
            new ParallelCommandGroup(
                mIntake.outtake(),
                new WaitCommand(0.2)),
            mNewElbow.Stow(),
            new ParallelCommandGroup(
                new RunCommand(
                    () -> mDriveSubsystem.drive(0.2, 0, 0, false, false),
                    mDriveSubsystem),
                new WaitCommand(3)
            ),
            new ParallelCommandGroup(
                new RunCommand(
                    () -> mDriveSubsystem.drive(-0.2, 0, 0, false, false),
                    mDriveSubsystem),
                new WaitCommand(1)
            ),
            new RunCommand(() -> oldBalance.balance())

        );
    }

}
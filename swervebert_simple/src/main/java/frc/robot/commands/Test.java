package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class Test extends SequentialCommandGroup {

    public Test(DriveSubsystem driveSubsystem){
        addRequirements(driveSubsystem);

        addCommands(
            new FollowPathWithEvents(null, null, null)
        );
    }
    
}

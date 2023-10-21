package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.subsystems.DriveSubsystem;

public class followPathWithEvents{
    HashMap<String, Command> eventMap = new HashMap<>();

    public followPathWithEvents(){
        eventMap.put("Intake", new IntakeCommand());
        eventMap.put("Outtake", null);
        eventMap.put("Balance", null);
    
    }

    public Command followPPPEvents(String name, DriveSubsystem driveSubsystem, double maxSpeed){
        PathPlannerTrajectory path = PathPlanner.loadPath(name, 
            maxSpeed,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    
        FollowPathWithEvents command = new FollowPathWithEvents(
            driveSubsystem.followPathCommand(path), 
            path.getMarkers(), 
            eventMap);
    
        return command;
    
      }
}
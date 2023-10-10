package frc.robot.actions;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class followPathWithEvents{
    HashMap<String, Command> eventMap = new HashMap<>();

    public followPathWithEvents(){
        eventMap.put("intake", null);
    
    }

    public Command followPPEvents(String name, DriveSubsystem driveSubsystem){
        PathPlannerTrajectory path = PathPlanner.loadPath(name, 
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    
        FollowPathWithEvents command = new FollowPathWithEvents(
            driveSubsystem.followPathCommand(path), 
            path.getMarkers(), 
            eventMap);

    
        return command;
    
      }
}
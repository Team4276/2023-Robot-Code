package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.ScoreLowBackCommand;
import frc.robot.subsystems.DriveSubsystem;

public class followPathWithEvents{
    private HashMap<String, Command> eventMap = new HashMap<>();

    public followPathWithEvents(){
        eventMap.put("intake", new IntakeCommand());
        eventMap.put("scorelowback", new ScoreLowBackCommand());
    
    }

    public Command followPPPEvents(String name, DriveSubsystem driveSubsystem, double maxSpeed){
        PathPlannerTrajectory path = PathPlanner.loadPath(name, 
            maxSpeed,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

        if (Robot.alliance != Alliance.Invalid){
            path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, Robot.alliance);
        }

        FollowPathWithEvents command = new FollowPathWithEvents(
        driveSubsystem.followPathCommand(path), 
        path.getMarkers(), eventMap);

        return command;
      }
}
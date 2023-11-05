package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.ScoreLowBackCommand;
import frc.robot.subsystems.DriveSubsystem;

public class followPathWithEvents{
    private HashMap<String, Command> eventMap = new HashMap<>();

    public followPathWithEvents(){
        eventMap.put("intake", new IntakeCommand());
        eventMap.put("scorelowback", new ScoreLowBackCommand());
        
        //SmartDashboard.putString("PP Alliance: ", AutoPicker.alliance.toString());

    
    }

    
    public Command followPPPEvents(String name, DriveSubsystem driveSubsystem, double maxSpeed){
        SmartDashboard.putString("PP DS Alliance", DriverStation.getAlliance().toString());

        PathPlannerTrajectory path = PathPlanner.loadPath(name, 
            maxSpeed,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

        //SmartDashboard.putString("PP Alliance: ", AutoPicker.alliance.toString());

        path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, Alliance.Red);

        FollowPathWithEvents command = new FollowPathWithEvents(
        driveSubsystem.followPathCommand(path), 
        path.getMarkers(), eventMap);

        return command;
      }


    public Command followPPPEvents(String name, DriveSubsystem driveSubsystem, double maxSpeed, Alliance alliance){
        PathPlannerTrajectory path = PathPlanner.loadPath(name, 
            maxSpeed,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

            path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, Alliance.Red);

        FollowPathWithEvents command = new FollowPathWithEvents(
        driveSubsystem.followPathCommand(path), 
        path.getMarkers(), eventMap);

        return command;
      }
}
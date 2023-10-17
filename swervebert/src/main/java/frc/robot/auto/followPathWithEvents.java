package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.actions.actions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NewElbow;

public class followPathWithEvents{
    HashMap<String, Command> eventMap = new HashMap<>();

    private final Intake mIntake = Intake.getInstance();
    private final NewElbow mNewElbow = NewElbow.getInstance();

    private actions actions = new actions();

    public followPathWithEvents(){
        eventMap.put("Intake", null);
        eventMap.put("Outtake", null);
        eventMap.put("Balance", null);
        eventMap.put("Elbow High", null);
        eventMap.put("Elbow Mid", null);
        eventMap.put("Elbow Low", null);
        eventMap.put("Elbow Intake", null);
        eventMap.put("action", new InstantCommand());
    
    }

    public Command followPPPEvents(String name, DriveSubsystem driveSubsystem){
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
package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPicker {
    public static SendableChooser<Command> chooser = new SendableChooser<Command>();

    HashMap<String, Command> eventMap = new HashMap<>();
    
    DriveSubsystem driveSubsystem;
    RobotContainer robotContainer;

    public AutoPicker(
        DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;

        chooser.setDefaultOption("Do nothing", null);
        chooser.addOption("Bump 2 Piece", followPathWithEvents("Bump 2 Piece"));
        chooser.addOption("Short test", followPathWithEvents("Short test"));
        chooser.addOption("No Rotate", followPathWithEvents("No Rotate"));
        chooser.addOption("Straight", followPathWithEvents("Straight"));
        chooser.addOption("WEEEE", followPathWithEvents("WEEEE"));

        eventMap.put("intake", new PrintCommand("Intaking"));

        SmartDashboard.putData("Auto Choices: ", chooser);

    }

    public Command getAutoCommand(){
        return chooser.getSelected();
    }

    private Command followPathWithEvents(String name){
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
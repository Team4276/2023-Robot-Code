package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPicker {
    public static SendableChooser<Command> chooser = new SendableChooser<Command>();

    HashMap<String, Command> eventMap = new HashMap<>();
    
    DriveSubsystem driveSubsystem;
    RobotContainer robotContainer;

    public AutoPicker(
        DriveSubsystem driveSubsystem,
        RobotContainer robotContainer){
        this.driveSubsystem = driveSubsystem;
        this.robotContainer = robotContainer;

        chooser.setDefaultOption("Do nothing", null);
        chooser.addOption("Bump 2 Piece", Bump2Piece());
        chooser.addOption("Short Test", ShortTest());

        eventMap.put("intake", new PrintCommand("Intaking"));

        SmartDashboard.putData("Auto Choices: ", chooser);

    }

    public Command getAutoCommand(){
        return chooser.getSelected();
    }

    private Command Bump2Piece() {
        PathPlannerTrajectory path = PathPlanner.loadPath("Bump 2 Piece", 3,3);

        FollowPathWithEvents command = new FollowPathWithEvents(
            robotContainer.getAutonomousCommand(path), 
            path.getMarkers(), 
            eventMap);

        return command;
        

    }

    private Command ShortTest() {
        PathPlannerTrajectory path = PathPlanner.loadPath("Short test", 3,3);

        FollowPathWithEvents command = new FollowPathWithEvents(
            robotContainer.getAutonomousCommand(path), 
            path.getMarkers(), 
            eventMap);

        return command;
        

    }

}
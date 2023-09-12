package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class AutoPicker {
    SendableChooser<Command> chooser = new SendableChooser<Command>();

    HashMap<String, Command> eventMap = new HashMap<>();

    
    private DriveSubsystem driveSubsystem;

    public AutoPicker(
        DriveSubsystem driveSubsystem,
        IntakeSubsystem intakeSubsystem,
        ArmSubsystem armSubsystem,
        LimeLightSubsystem limeLightSubsystem,
        XboxController xboxController){
        this.driveSubsystem = driveSubsystem;

        chooser.setDefaultOption("Do nothing", null);
        chooser.addOption("Bump 2 Piece", followPathWithEvents("Bump 2 Piece"));
        chooser.addOption("Short test", followPathWithEvents("Short test"));
        chooser.addOption("No Rotate", followPathWithEvents("No Rotate"));
        chooser.addOption("Straight", followPathWithEvents("Straight"));
        chooser.addOption("WEEEE", followPathWithEvents("WEEEE"));

        eventMap.put("balance", new Balance(driveSubsystem, armSubsystem, intakeSubsystem));
        eventMap.put("scoreConeHigh", new ScoreConeHigh(armSubsystem, intakeSubsystem, driveSubsystem, limeLightSubsystem, xboxController, false));
        eventMap.put("scoreConeMid", new ScoreConeHigh(armSubsystem, intakeSubsystem, driveSubsystem, limeLightSubsystem, xboxController, false));
        eventMap.put("scoreConeLow", new ScoreConeHigh(armSubsystem, intakeSubsystem, driveSubsystem, limeLightSubsystem, xboxController, false));
        eventMap.put("scoreCubeHigh", new ScoreConeHigh(armSubsystem, intakeSubsystem, driveSubsystem, limeLightSubsystem, xboxController, false));
        eventMap.put("scoreCubeMid", new ScoreConeHigh(armSubsystem, intakeSubsystem, driveSubsystem, limeLightSubsystem, xboxController, false));
        eventMap.put("scoreCubeLow", new ScoreConeHigh(armSubsystem, intakeSubsystem, driveSubsystem, limeLightSubsystem, xboxController, false));


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
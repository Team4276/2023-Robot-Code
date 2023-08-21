package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoPicker {
    SendableChooser<Command> chooser = new SendableChooser<Command>();

    HashMap<String, Command> eventMap = new HashMap<>();

    
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;

    public AutoPicker(
        DriveSubsystem driveSubsystem,
        IntakeSubsystem intakeSubsystem,
        ArmSubsystem armSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;

        chooser.setDefaultOption("Do nothing", null);
        chooser.addOption("Bump 2 Piece", followPathWithEvents("Bump 2 Piece"));
        chooser.addOption("Short test", followPathWithEvents("Short test"));
        chooser.addOption("No Rotate", followPathWithEvents("No Rotate"));
        chooser.addOption("Straight", followPathWithEvents("Straight"));
        chooser.addOption("WEEEE", followPathWithEvents("WEEEE"));

        eventMap.put("intake", intakeSubsystem.in());
        eventMap.put("outtake", intakeSubsystem.out());
        eventMap.put("balance", new Balance(driveSubsystem, armSubsystem, intakeSubsystem));

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
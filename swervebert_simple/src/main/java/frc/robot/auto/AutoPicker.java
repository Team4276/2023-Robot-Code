package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PIDElbow;

public class AutoPicker {
    SendableChooser<Command> chooser = new SendableChooser<Command>();

    HashMap<String, Command> eventMap = new HashMap<>();
    
    DriveSubsystem driveSubsystem;
    PIDElbow pidElbow;
    Intake intake;

    public AutoPicker(
        DriveSubsystem driveSubsystem,
        PIDElbow pidElbow,
        Intake intake){
        this.driveSubsystem = driveSubsystem;
        this.pidElbow = pidElbow;
        this.intake = intake;

        eventMap.put("intake", new PrintCommand("Intaking"));

        chooser.setDefaultOption("Do nothing", null);
        chooser.addOption(null, getAutoCommand());

        SmartDashboard.putData("Auto Test: ", chooser);

    }

    public Command getAutoCommand(){
        return chooser.getSelected();
    }

    public Command followPathWithEvents(String name){
        PathPlannerTrajectory path = PathPlanner.loadPath(name, 
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    
        FollowPathWithEvents command = new FollowPathWithEvents(
            driveSubsystem.followPathCommand(path), 
            path.getMarkers(), 
            eventMap);
    
        return command;
    
      }

    //TODO: Rework the paths to be segmented like 254
    //TODO: look into auto architecture
    //TODO: give heading control to the code

   


}
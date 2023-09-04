package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Balance extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;
    
    public Balance(DriveSubsystem driveSubsystem, 
        ArmSubsystem armSubsystem, 
        IntakeSubsystem intakeSubsystem){
        
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        driveSubsystem.getCurrentCommand().end(true);
        armSubsystem.getCurrentCommand().end(true);
        intakeSubsystem.getCurrentCommand().end(true);

        armSubsystem.stow();
        intakeSubsystem.idle();
    }

    @Override
    public void execute() {
        if(driveSubsystem.m_bgyro.GetRoll() > 12){
            driveSubsystem.drive(0.1, 0, 0, 
                true, true, 
                DriveConstants.kMaxSpeedMetersPerSecond);

        } else if (driveSubsystem.m_bgyro.GetRoll() < -12){
            driveSubsystem.drive(-0.1, 0, 0, 
                true, true, 
                DriveConstants.kMaxSpeedMetersPerSecond);
        } else {
            driveSubsystem.drive(0, 0, 0, 
                false, false, 
                DriveConstants.kMaxSpeedMetersPerSecond);

            this.end(false);
        }
    
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setX();
    
    }
    
}

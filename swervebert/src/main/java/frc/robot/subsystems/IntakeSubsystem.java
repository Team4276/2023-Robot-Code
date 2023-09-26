package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmSubsystemConstants;
import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax intake;

    public IntakeSubsystem(){
        intake = new CANSparkMax(ArmSubsystemConstants.elbowMotorCanId, MotorType.kBrushless);
    }

    public void out(){
        intake.set(IntakeSubsystemConstants.kOuttakeSpeed);
    }

    public void in(){
        intake.set(IntakeSubsystemConstants.kIntakeSpeed);
    }

    public void idle(){
        intake.set(IntakeSubsystemConstants.kIdleSpeed);
    }
    
}

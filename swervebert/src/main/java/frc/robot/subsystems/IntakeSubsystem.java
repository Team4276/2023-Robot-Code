package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax intake;

    public IntakeSubsystem(){
        intake = new CANSparkMax(ArmSubsystemConstants.elbowMotorCanId, MotorType.kBrushless);

    }

    public Command out(){
        return new InstantCommand(() -> { intake.set(-0.85);});
    }

    public Command in(){
        return new InstantCommand(() -> { intake.set(0.85);});
    }

    public Command idle(){
        return new InstantCommand(() -> { intake.set(0.05);});
    }
    
}

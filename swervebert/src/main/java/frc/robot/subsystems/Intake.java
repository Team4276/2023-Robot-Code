package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private static CANSparkMax intakeMotor;

    private static Intake mInstance = null;
    public static Intake getInstance(){
        if(mInstance == null){
            mInstance = new Intake();
        }

        return mInstance;
    }

    private Intake() {
        intakeMotor = new CANSparkMax(IntakeConstants.IntakeID, MotorType.kBrushless);
    }

    public Command intake(){
        return new InstantCommand(() -> {intakeMotor.set(IntakeConstants.intakeSpeed);}); 
    }

    public Command outtake(){
        return new InstantCommand(() -> {intakeMotor.set(IntakeConstants.outtakeSpeed);}); 
    }

    public Command idle(){
        return new InstantCommand(() -> {intakeMotor.set(IntakeConstants.idleSpeed);}); 
    }
}

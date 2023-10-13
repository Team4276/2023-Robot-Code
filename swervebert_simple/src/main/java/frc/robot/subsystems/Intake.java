package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private static CANSparkMax intakeDrive;

    public Intake(int port) {
        intakeDrive = new CANSparkMax(port, MotorType.kBrushless);
    }

    // Speed inrange -1.0 to +1.0
    public void setSpeed(double speed) {
        intakeDrive.set(speed);
    }

    public void intake(){
        setSpeed(0.8);
    }

    public void outtake(){
        setSpeed(-0.85);
    }

    public void idle(){
        setSpeed(0.05);
    }
}

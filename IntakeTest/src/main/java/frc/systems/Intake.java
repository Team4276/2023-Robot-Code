package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake {

    private static CANSparkMax intakeDrive;

    public Intake(int port) {
        intakeDrive = new CANSparkMax(port, MotorType.kBrushless);
    }    

    // Speed inrange -1.0 to +1.0
    public void setSpeed(double speed) {
        intakeDrive.set(speed);
    }
}

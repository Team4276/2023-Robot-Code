package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

public class Wrist {
    
    private static CANSparkMax driveWrist;
    private double deadband = 0.05;


    public Wrist(int port) {
        driveWrist = new CANSparkMax(port, MotorType.kBrushless);
   }    

    // Speed inrange -1.0 to +1.0
    public void setWristSpeed(double speed) {
        driveWrist.set(speed);
     }
    
    public void updatePeriodic() {
        if (Math.abs(Robot.xboxController.getRightY()) > deadband) {
            double rightY = Math.pow(Robot.xboxController.getRightY(), 3 / 2);
            setWristSpeed(rightY);
        }
    }
}

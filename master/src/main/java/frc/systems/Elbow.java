package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

public class Elbow {

    public static CANSparkMax driveElbow;
    private double deadband = 0.2;

    public Elbow(int port) {
        driveElbow = new CANSparkMax(port, MotorType.kBrushless);
        setElbowSpeed(0);
    }

    // Speed inrange -1.0 to +1.0
    public void setElbowSpeed(double speed) {
        driveElbow.set(speed);
    }

    public boolean updatePeriodic() {
        if (Math.abs(Robot.xboxController.getRightY()) > deadband) {
            double rightY = Math.pow(Robot.xboxController.getRightY(), 3 / 2);
            setElbowSpeed(rightY);
            return true;
        } else {
            setElbowSpeed(0);
            return false;
        }
    }
}

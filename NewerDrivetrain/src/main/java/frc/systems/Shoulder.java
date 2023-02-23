package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

public class Shoulder {
    
    private static CANSparkMax driveShoulder_R;
    private static CANSparkMax driveShoulder_L;
    private double deadband = 0.05;


    public Shoulder(int port_R, int port_L) {
        driveShoulder_R = new CANSparkMax(port_R, MotorType.kBrushless);
        driveShoulder_L = new CANSparkMax(port_L, MotorType.kBrushless);
    }    

    // Speed inrange -1.0 to +1.0
    public void setShoulderSpeed_R(double speed) {
        driveShoulder_R.set(speed);
     }
    public void setShoulderSpeed_L(double speed) {
        driveShoulder_L.set(speed);
    }

    public void updatePeriodic() {
        if (Math.abs(Robot.xboxController.getLeftY()) > deadband) {
            double leftY = Math.pow(Robot.xboxController.getLeftY(), 3 / 2);
            Robot.mShoulder.setShoulderSpeed_R(leftY);
            Robot.mShoulder.setShoulderSpeed_L(leftY);
        }
        if (Math.abs(Robot.xboxController.getLeftY()) > deadband) {
            double leftY = -Math.pow(Robot.xboxController.getLeftY(), 3 / 2);
            Robot.mShoulder.setShoulderSpeed_L(leftY);
        }
    }
}

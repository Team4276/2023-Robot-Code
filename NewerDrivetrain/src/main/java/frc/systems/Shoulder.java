package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Shoulder {
    private static double leftY;
    private static CANSparkMax driveShoulder_R;
    private static CANSparkMax driveShoulder_L;
    private double deadband = 0.05;

    public Shoulder(int port_R, int port_L) {
        driveShoulder_R = new CANSparkMax(port_R, MotorType.kBrushless);
        driveShoulder_L = new CANSparkMax(port_L, MotorType.kBrushless);
        setShoulderSpeed_R(0);
        setShoulderSpeed_L(0);
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
            leftY = Math.pow(Robot.xboxController.getLeftY(), 3 / 2);
            setShoulderSpeed_R(-1 * leftY);
            setShoulderSpeed_L(leftY);
        } else {
            setShoulderSpeed_R(0);
            setShoulderSpeed_L(0);
        }

        SmartDashboard.putNumber("Shoulder Power", leftY);
    }
}

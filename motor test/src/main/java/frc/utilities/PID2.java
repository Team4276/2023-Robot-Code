package frc.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Robot;

public class PID2 {
    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;

    private double kMaxOutput = 1;
    private double kMinOutput = -1;

    PIDController pid = new PIDController(Kp, Ki, Kd);

    double testPIDOutput = MathUtil.clamp(pid.calculate(Robot.leftJoystick.getY(), 0), kMinOutput, kMaxOutput);


    
}

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utilities.Xbox;

public class Test {
    public static void test(){
        double Power = Robot.leftJoystick.getY();
        Power *= 10;
        SmartDashboard.putNumber("Set Position", Power);
    }
    
    public static void test2(){
        if (Robot.xboxController.getRawButton(Xbox.B)){
            double testpoint = -20;
            SmartDashboard.putNumber("Set Position", testpoint);
        } else if (Robot.xboxController.getRawButton(Xbox.A)){
            double testpoint = 0;
            SmartDashboard.putNumber("Set Position", testpoint);
        }
    }
}

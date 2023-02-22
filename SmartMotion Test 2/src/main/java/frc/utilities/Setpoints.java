package frc.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

public class Setpoints {
    public static void velocity(){
        double Power = Robot.leftJoystick.getY();
        Power *= 100;
        SmartDashboard.putNumber("Set Velocity", Power);
    }

    public static boolean dpad(double pov){
        if (Xbox.POVup == pov){
            double testpoint = 10;
            SmartDashboard.putNumber("Set Position", testpoint);
            return false;
        } else if (Xbox.POVdown == pov){
            double testpoint = -10;
            SmartDashboard.putNumber("Set Position", testpoint);
            return false;
        } else if (Xbox.POVright == pov){
            double testpoint = 5;
            SmartDashboard.putNumber("Set Position", testpoint);
            return false;
        } else if (Xbox.POVleft == pov){
            double testpoint = -5;
            SmartDashboard.putNumber("Set Position", testpoint);
            return false;
        }
        return true;
    }
}

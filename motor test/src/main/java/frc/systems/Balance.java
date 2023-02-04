package frc.systems;

import frc.utilities.PID;
import frc.utilities.Xbox;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Balance {
    private static double DEAD_ZONE = 3;

    public static double power;

    public static void balance(double pitch){
        if (Robot.xboxController.getRawButton(Xbox.B)){
            if (Math.abs(pitch) > DEAD_ZONE ){
                power = PID.getOutput(pitch, 0);
                AutoDrivetrain.usingAutoDrivetrain = true;
                SmartDashboard.putNumber("Set Velocity", power);
            } else {
                AutoDrivetrain.holdPosition = true;
            }
        }
    }
}

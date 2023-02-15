package frc.systems;

import frc.utilities.PID;
import frc.utilities.Xbox;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Balance {
    private static double DEAD_ZONE = 1;
    private static boolean stop = false;

    public static void balance(double pitch){
        SmartDashboard.putBoolean("Stop", stop);
        if (Robot.xboxController.getRawButton(Xbox.B)){    
            if (stop){
                AutoDrivetrain.holdPosition = true;
                BaseDrivetrain.usingAutoDrivetrain = true;
            } else {
                if (Math.abs(pitch) > DEAD_ZONE ){
                    AutoDrivetrain.holdPosition = false;
                    BaseDrivetrain.usingAutoDrivetrain = true;
                    AutoDrivetrain.setPoint = PID.getOutput(pitch, 0);
                } else {
                    stop = true;
                }
            }
        } else {
            BaseDrivetrain.usingAutoDrivetrain = false;
            AutoDrivetrain.holdPosition = false;
            stop = false;
        }
    }
}

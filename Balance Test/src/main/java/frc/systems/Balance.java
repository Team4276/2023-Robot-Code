package frc.systems;

import frc.utilities.PID;
import frc.utilities.Xbox;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Balance {
    private static double DEAD_ZONE = 1;
    private static double stop = 0;
    private static double power;
    private static double aVelocity = 1000;

    private static boolean mode = true;
    private static boolean aligned = false;
    
    private static Timer timer;

    private static double bound1 = 0;
    private static double bound2 = 0;
    private static double middle = 0;
    private static double allowedErr = 1;

    public static void balanceInit(){
        timer = new Timer();
        timer.start();
    }

    public static void balance(double pitch){
        if (Robot.xboxController.getRawButton(Xbox.B)){
            if (mode) {            
                if (stop == 0){
                    if (Math.abs(pitch) > DEAD_ZONE ){
                        AutoDrivetrain.holdPosition = false;
                        power = PID.getOutput(pitch, 0);
                        AutoDrivetrain.usingAutoDrivetrain = true;
                        SmartDashboard.putNumber("Set Velocity", power);
                    } else {
                        if (aligned != true){
                            stop = timer.get() + 3;
                        }
                        AutoDrivetrain.holdPosition = true;
                    }
                } else if (timer.get() == stop) {
                        mode = false;
                }
            } else {
                if (AutoDrivetrain.FR_encoder.getPosition() > (middle + allowedErr)){
                    if (AutoDrivetrain.FL_encoder.getPosition() < middle - allowedErr){
                        AutoDrivetrain.holdPosition = true;
                        aligned = true;
                        mode = true;
                    }
                } else if (pitch > DEAD_ZONE){
                    bound1 = AutoDrivetrain.FR_encoder.getPosition();
                } else if (pitch < DEAD_ZONE) {
                    bound2 = AutoDrivetrain.FR_encoder.getPosition();
                    middle = (bound2 + Math.abs(bound1-bound2));
                } else {
                    if (bound1 == 0){
                        AutoDrivetrain.holdPosition = false;
                        AutoDrivetrain.usingAutoDrivetrain = true;
                        SmartDashboard.putNumber("Set Velocity", aVelocity);
                    } else if (bound2 == 0){
                        AutoDrivetrain.holdPosition = false;
                        AutoDrivetrain.usingAutoDrivetrain = true;
                        SmartDashboard.putNumber("Set Velocity", -aVelocity);
                    } else {
                        AutoDrivetrain.holdPosition = false;
                        AutoDrivetrain.usingAutoDrivetrain = true;
                        SmartDashboard.putBoolean("Mode", false);
                        SmartDashboard.putNumber("Set Position", middle);
                    }
                }
            }


        } else {
            AutoDrivetrain.usingAutoDrivetrain = false;
            AutoDrivetrain.holdPosition = false;
            mode = true;
            aligned = false;
            bound1 = 0;
            bound2 = 0;
            middle = 0;
            allowedErr = 1;
            stop = 0;

        }
    }
}

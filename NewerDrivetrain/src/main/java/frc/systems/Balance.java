package frc.systems;

import frc.utilities.PID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Balance {
    private static double DEAD_ZONE = 1;
    private static boolean stop = false;

    public static void balance(double pitch) {
        SmartDashboard.putBoolean("Stop", stop);
        if (stop) {
            PIDDrivetrain.holdPosition = true;
        } else {
            if (Math.abs(pitch) > DEAD_ZONE) {
                PIDDrivetrain.holdPosition = false;
                PIDDrivetrain.setPoint = PID.getOutput(pitch, 0);
            } else {
                stop = true;
            }
        }
    }

    public static void stopBalance() {
        PIDDrivetrain.holdPosition = false;
        stop = false;
    }
}

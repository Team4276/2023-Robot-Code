package frc.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utilities.PID;

public class Balance {
    private static double DEAD_ZONE = 1;

    public static void balance(double pitch) {
        boolean pause = false;
        SmartDashboard.putBoolean("Stop", pause);
        if (pause) {
            PIDDrivetrain.holdPosition = true;
        } else {
            if (Math.abs(pitch) > DEAD_ZONE) {
                PIDDrivetrain.holdPosition = false;
                PIDDrivetrain.mode = true;
                PIDDrivetrain.setPoint = PID.getOutput(pitch, 0);
            } else {
                pause = true;
            }
        }
    }
}

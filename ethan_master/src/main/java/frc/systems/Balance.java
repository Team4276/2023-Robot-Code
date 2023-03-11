package frc.systems;

import frc.utilities.PID;

public class Balance {
    private static double DEAD_ZONE = 2;

    public static void balance(double pitch) {
        boolean pause = false;
        if (pause) {
            TeleopDrivetrain.assignMotorPower(0, 0);
        } else {
            if (Math.abs(pitch) > DEAD_ZONE) {
                PIDDrivetrain.holdPosition = false;
                PIDDrivetrain.setPoint = (PID.getOutput(pitch, 0))/4;
            } else {
                pause = true;
            }
        }
    }
}

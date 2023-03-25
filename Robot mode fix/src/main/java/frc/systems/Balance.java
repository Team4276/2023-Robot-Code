package frc.systems;

import frc.utilities.PID;
import frc.utilities.SoftwareTimer;

public class Balance {
    private static final double DEAD_ZONE = 3;
    private static final double CHECKTIME = 2;
    public static boolean pause = false;
    private static SoftwareTimer checkTimer;

    private static boolean firstRun = true;

    public static void balanceinit() {
        checkTimer = new SoftwareTimer();
    }

    public static void balance(double pitch) {
        if (pause) {
            if (firstRun) {
                checkTimer.setTimer(CHECKTIME);
                firstRun = false;
            }

            if (checkTimer.isExpired()) {
                pause = false;
            } else {
                TeleopDrivetrain.assignMotorPower(0, 0);

            }

        } else {
            if (Math.abs(pitch) > DEAD_ZONE) {
                PIDDrivetrain.setPoint = (PID.getOutput(pitch, 0)) / 4;
            } else {
                pause = true;
            }

            firstRun = true;
        }
    }
}

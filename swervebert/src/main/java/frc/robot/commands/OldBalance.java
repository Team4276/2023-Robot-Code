package frc.robot.commands;

import edu.wpi.first.math.MathUtil;

import frc.robot.subsystems.DriveSubsystem;

import frc.utils.PID;
import frc.utils.SoftwareTimer;

public class OldBalance {
    private static double DEAD_ZONE = 3;
    public static boolean pause = false;
    private static final double CHECKTIME = 2;
    private static SoftwareTimer checkTimer;

    DriveSubsystem driveSubsystem;

    private static boolean firstRun = true;

    // must be oriented to face the charge station
    //TODO: change it to go sideways?

    public OldBalance(DriveSubsystem driveSubsystem){
        checkTimer = new SoftwareTimer();

        this.driveSubsystem = driveSubsystem;
    }

    public void balance() {
        double pitch = driveSubsystem.getPitch();

        if (pause) {
            if (firstRun) {
                checkTimer.setTimer(CHECKTIME);
                firstRun = false;
            }

            if (checkTimer.isExpired()) {
                pause = false;
            } else {
                driveSubsystem.drive(0, 0, 0, false, false);

            }

        } else {
            if (Math.abs(pitch) > DEAD_ZONE) {
                double speed = -MathUtil.clamp(PID.getOutput(pitch, 0) / 4, -0.5, 0.5);

                driveSubsystem.drive(0, speed, 0, false, false);
            } else {
                pause = true;
            }

            firstRun = true;
        }
    }
}

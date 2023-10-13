package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PID;
import frc.utils.SoftwareTimer;

//TODO: fix this crap

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

                driveSubsystem.drive(0, -PID.getOutput(pitch, 0) / 4, 0, false, false);
            } else {
                pause = true;
            }

            firstRun = true;
        }
    }
}

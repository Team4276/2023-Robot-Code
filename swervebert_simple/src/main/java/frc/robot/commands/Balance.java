package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.utils.SoftwareTimer;

public class Balance {
    private double DEAD_ZONE = 3;
    public boolean pause = false;
    private final double CHECKTIME = 2;
    private SoftwareTimer checkTimer;

    private boolean firstRun = true;

    DriveSubsystem driveSubsystem;

    public void balanceinit(DriveSubsystem driveSubsystem) {
        checkTimer = new SoftwareTimer();

        this.driveSubsystem = driveSubsystem;
    }

    public void balance(double pitch) {
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

                // TODO: orient it to the balance;
                driveSubsystem.drive(0, 0.1, 0, false, false);
                
            } else {
                pause = true;
            }

            firstRun = true;
        }
    }
}

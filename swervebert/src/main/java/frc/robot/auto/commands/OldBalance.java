package frc.robot.auto.commands;

import edu.wpi.first.math.MathUtil;

import frc.robot.subsystems.DriveSubsystem;

import frc.utils.SoftwareTimer;

public class OldBalance {
    private static double DEAD_ZONE = 3;
    public static boolean pause = false;
    private static final double CHECKTIME = 2;
    private static SoftwareTimer checkTimer;

    private DriveSubsystem driveSubsystem;

    private static boolean firstRun = true;

    public OldBalance(){
        driveSubsystem = DriveSubsystem.getInstance();

        checkTimer = new SoftwareTimer();
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
                double speed = MathUtil.clamp(pitch * 0.01, -0.5, 0.5);

                driveSubsystem.drive(speed, 0, 0, false, false);
            } else {
                pause = true;
            }

            firstRun = true;
        }
    }
}

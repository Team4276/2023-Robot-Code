package frc.auto;

import frc.systems.TeleopDrivetrain;
import frc.utilities.SoftwareTimer;

public class BabyAuto {
    private static double leftMobilityTime = 0;
    private static double rightMobilityTime = 0;

    private static SoftwareTimer timer;

    private static boolean firstRun = true;

    public static boolean taskIsFinished = false;

    public static void leftScoreMobility(){
        if (firstRun){
            timer.setTimer(leftMobilityTime);
            firstRun = false;
        }

        if (timer.isExpired()){
            TeleopDrivetrain.assignMotorPower(0,0);
            taskIsFinished = true;
        } else {
            TeleopDrivetrain.assignMotorPower(-1 * 0.5,0.5);
        }

    }

    public static void middleScoreBalance(){

    }

    public static void rightScoreMobility(){
        if (firstRun){
            timer.setTimer(rightMobilityTime);
            firstRun = false;
        }

        if (timer.isExpired()){
            TeleopDrivetrain.assignMotorPower(0,0);
            taskIsFinished = true;
        } else {
            TeleopDrivetrain.assignMotorPower(-1 * 0.5,0.5);
        }

        
    }
}

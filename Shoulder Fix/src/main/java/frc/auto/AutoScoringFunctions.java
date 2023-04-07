package frc.auto;

import frc.utilities.SoftwareTimer;

public class AutoScoringFunctions {
    public static boolean usingIntake = false;
    public static boolean usingArm = true;
    public static boolean taskIsFinished = false;

    private static boolean firstRun = true;
    
    private static final double INTAKERUNTIME = 1;

    private static SoftwareTimer intakeTimer;

    public static void AutoScoringFunctionsInit(){
        intakeTimer = new SoftwareTimer();
    }

    public static void scoreLow(){
        if (firstRun) {
            intakeTimer.setTimer(INTAKERUNTIME);
            firstRun = false;
        }

        if (intakeTimer.isExpired()){
            usingIntake = false;
            taskIsFinished = true;

        } else {
            usingIntake = true;
        }

    }


}

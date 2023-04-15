package frc.auto;

import frc.systems.PIDElbow;
import frc.utilities.SoftwareTimer;

public class AutoScoringFunctions {
    public static boolean usingIntake = false;
    public static boolean usingArm = true;
    public static boolean taskIsFinished = false;

    private static boolean firstRun1 = true;
    private static boolean firstRun2 = true;
    
    private static final double INTAKERUNTIME = 1;
    private static final double ARMRUNTIME1 = 1; // end approximately when it gets to score position
    private static final double ARMRUNTIME2 = 1; // end approximately when it gets to stow position

    private static SoftwareTimer intakeTimer;
    private static SoftwareTimer armTimer1;
    private static SoftwareTimer armTimer2;

    public static void AutoScoringFunctionsInit(){
        intakeTimer = new SoftwareTimer();
        armTimer1 = new SoftwareTimer();
        armTimer2 = new SoftwareTimer();
    }

    public static void scoreLow(){
        if (firstRun1) {
            intakeTimer.setTimer(INTAKERUNTIME);
            firstRun1 = false;
        }

        if (intakeTimer.isExpired()){
            usingIntake = false;
            taskIsFinished = true;

        } else {
            usingIntake = true;
        }

    }

    public static void scoreMid(){
        if (firstRun1){
            PIDElbow.setPoint_Elbow = 0 + PIDElbow.elbowZero; //placeholder for score
            armTimer1.setTimer(ARMRUNTIME1);
            firstRun1 = false;
            intakeTimer.setTimer(ARMRUNTIME1 + INTAKERUNTIME);
        }

        if (armTimer2.isExpired()){
            usingIntake = true;
            if (firstRun2){
                PIDElbow.setPoint_Elbow = 0 + PIDElbow.elbowZero; // placeholder for stow
                firstRun2 = false;
            }

            if (intakeTimer.isExpired()){
                usingIntake = false;
            }

        }

        


    }

    


}

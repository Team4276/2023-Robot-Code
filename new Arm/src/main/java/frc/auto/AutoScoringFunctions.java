package frc.auto;

import frc.systems.PIDElbow;
import frc.systems.PIDShoulder;
import frc.utilities.SoftwareTimer;

public class AutoScoringFunctions {
    public static boolean usingIntake = false;
    public static boolean usingArm = true;
    public static boolean taskIsFinished = false;

    private static boolean firstRun = true;
    
    private static final double DEADZONE = 0.5;
    private static final double INTAKERUNTIME = 2;
    private static final double ARMRUNTIME = 4;

    private static SoftwareTimer intakeTimer;
    private static SoftwareTimer armTimer;

    public static void AutoScoringFunctionsInit(){
        intakeTimer = new SoftwareTimer();
        armTimer = new SoftwareTimer();
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

    public static void scoreNearCube(){
        PIDElbow.setPoint_Elbow = PIDElbow.DPAD_LEFT_ELBOW_EJECT_CUBE;
        PIDShoulder.setPoint_Shoulder = PIDShoulder.DPAD_LEFT_SHOULDER_EJECT_CUBE;
        if ((Math.abs(PIDElbow.setPoint_Elbow - PIDElbow.driveElbow.getEncoder().getPosition()) < DEADZONE)
            && ((Math.abs(PIDShoulder.setPoint_Shoulder - PIDShoulder.driveShoulder_L.getEncoder().getPosition()) < DEADZONE)
            || (Math.abs(PIDShoulder.setPoint_Shoulder - PIDShoulder.driveShoulder_R.getEncoder().getPosition())) < DEADZONE)){
            usingIntake = true;
            if (firstRun){
                intakeTimer.setTimer(INTAKERUNTIME);
                firstRun = false;
            }
        } else {
            usingIntake = false;
        }

        if(intakeTimer.isExpired()) {
            usingIntake = false;
            firstRun = true;
            taskIsFinished = true;
            
        }
        

    }

    public static void safeScoreNearCube(){
        PIDElbow.setPoint_Elbow = PIDElbow.DPAD_LEFT_ELBOW_EJECT_CUBE;
        PIDShoulder.setPoint_Shoulder = PIDShoulder.DPAD_LEFT_SHOULDER_EJECT_CUBE;
        if (firstRun){
            armTimer.setTimer(ARMRUNTIME);
        }
        if (armTimer.isExpired()){
            usingIntake = true;
            if (firstRun){
                intakeTimer.setTimer(INTAKERUNTIME);
                firstRun = false;
            }
        } else {
            usingIntake = false;
        }

        if(intakeTimer.isExpired()) {
            usingIntake = false;
            firstRun = true;
            taskIsFinished = true;
            
        }
        

    }

    public static void scoreNearCone(){
        PIDElbow.setPoint_Elbow = PIDElbow.DPAD_RIGHT_ELBOW_REACH_NEAR_CONE;
        PIDShoulder.setPoint_Shoulder = PIDShoulder.DPAD_RIGHT_SHOULDER_REACH_NEAR_CONE;
        if ((Math.abs(PIDElbow.setPoint_Elbow - PIDElbow.driveElbow.getEncoder().getPosition()) < DEADZONE)
        && ((Math.abs(PIDShoulder.setPoint_Shoulder - PIDShoulder.driveShoulder_L.getEncoder().getPosition()) < DEADZONE)
        || (Math.abs(PIDShoulder.setPoint_Shoulder - PIDShoulder.driveShoulder_R.getEncoder().getPosition())) < DEADZONE)){
            usingIntake = true;
            if (firstRun){
                intakeTimer.setTimer(INTAKERUNTIME);
                firstRun = false;
            }
        } else {
            usingIntake = false;
        }

        if(intakeTimer.isExpired()) {
            usingIntake = false;
            firstRun = true;
            taskIsFinished = true;
            
        }

    }

    public static void safeScoreNearCone(){
        PIDElbow.setPoint_Elbow = PIDElbow.DPAD_RIGHT_ELBOW_REACH_NEAR_CONE;
        PIDShoulder.setPoint_Shoulder = PIDShoulder.DPAD_RIGHT_SHOULDER_REACH_NEAR_CONE;
        if (firstRun){
            armTimer.setTimer(ARMRUNTIME);
        }
        if (armTimer.isExpired()){
            usingIntake = true;
            if (firstRun){
                intakeTimer.setTimer(INTAKERUNTIME);
                firstRun = false;
            }
        } else {
            usingIntake = false;
        }

        if(intakeTimer.isExpired()) {
            usingIntake = false;
            firstRun = true;
            taskIsFinished = true;
            
        }
        

    }

    public static void scoreFarCube(){

    }

    public static void scoreFarCone(){
        
    }
}

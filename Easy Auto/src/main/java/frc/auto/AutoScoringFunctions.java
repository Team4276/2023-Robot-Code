package frc.auto;

import frc.systems.PIDElbow;
import frc.systems.PIDShoulder;
import frc.utilities.SoftwareTimer;

public class AutoScoringFunctions {
    public static boolean usingIntake = false;
    
    private static double DEADZONE = 0.5;

    public static void scoreLow(){

    }

    public static void scoreNearCube(){
        PIDElbow.setPoint_Elbow = PIDElbow.DPAD_RIGHT_ELBOW_EJECT_CUBE;
        PIDShoulder.setPoint_Shoulder = PIDShoulder.DPAD_RIGHT_SHOULDER_EJECT_CUBE;
        if ((Math.abs(PIDElbow.setPoint_Elbow - PIDElbow.driveElbow.getEncoder().getPosition()) < DEADZONE)
            && (Math.abs(PIDShoulder.setPoint_Shoulder - PIDShoulder.driveShoulder_L.getEncoder().getPosition()) < DEADZONE)
            && (Math.abs(PIDShoulder.setPoint_Shoulder - PIDShoulder.driveShoulder_R.getEncoder().getPosition()) < DEADZONE)){
            
            
            usingIntake = true;
        } else if() {
            usingIntake = false;
        } else {
            usingIntake = false;
        }
        

    }

    public static void scoreNearCone(){
        PIDElbow.setPoint_Elbow = PIDElbow.DPAD_UP_ELBOW_REACH_NEAR_CONE;
        PIDShoulder.setPoint_Shoulder = PIDShoulder.DPAD_UP_SHOULDER_REACH_NEAR_CONE;
        if ((Math.abs(PIDElbow.setPoint_Elbow - PIDElbow.driveElbow.getEncoder().getPosition()) < DEADZONE)
            && (Math.abs(PIDShoulder.setPoint_Shoulder - PIDShoulder.driveShoulder_L.getEncoder().getPosition()) < DEADZONE)
            && (Math.abs(PIDShoulder.setPoint_Shoulder - PIDShoulder.driveShoulder_R.getEncoder().getPosition()) < DEADZONE)){
            usingIntake = true;
        } else if() {
            usingIntake = false;
        } else {
            usingIntake = false;
        }

    }

    public static void scoreFarCube(){

    }

    public static void scoreFarCone(){
        
    }
}

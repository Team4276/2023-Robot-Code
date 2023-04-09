package frc.auto;

import frc.robot.Robot;
import frc.systems.TeleopDrivetrain;
import frc.utilities.SoftwareTimer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BabyAuto {
    private static final double LEFTMOBILITYTIME = 4.5; // time in seconds
    private static final double RIGHTMOBILITYTIME = 2.5; // time in seconds
    private static final double MIDDLEBALANCTIME1 = 0.5; // initial back away
    private static final double MIDDLEBALANCTIME2 = 2.5; // drive up onto platform
    private static final double MIDDLEBALANCTIME3 = 0; // delay until arm goes down
    public static final double MOTORPOWER = 0.20;
    private static final double DEADZONEANGLE = 5; // deg

    public static boolean balance = false;

    private static SoftwareTimer timer;
    private static SoftwareTimer middleTimer;
    private static SoftwareTimer middleArmDelayTimer;
    private static SoftwareTimer timer4_GOD_IGOTTASTOPADDINSOMANYTIMERS;

    private static boolean firstRun = true;
    private static boolean firstRunTimer = true;
    private static boolean firstRunTimer3_holy_cow_we_have_too_many_logic_variables_for_this_code_i_ned_to_fix = true;

    public static boolean taskIsFinished = false;

    public static boolean usingDrivetrainMotorsForward = false;
    public static boolean usingDrivetrainMotorsBackward = false;
    public static boolean usingDrivetrainMotorsNOPOWER = false;

    public static void BabyAutoInit(){
        timer = new SoftwareTimer();
        middleTimer = new SoftwareTimer();
        middleArmDelayTimer = new SoftwareTimer();
        timer4_GOD_IGOTTASTOPADDINSOMANYTIMERS = new SoftwareTimer();
    }

    public static void leftScoreMobility(){
        if (firstRun){
            timer.setTimer(LEFTMOBILITYTIME);
            firstRun = false;
        }

        if (timer.isExpired()){
            usingDrivetrainMotorsNOPOWER = true;
            taskIsFinished = true;
        } else {
            usingDrivetrainMotorsBackward = true;
        }
    }

    public static void middleBalance(boolean forward){
        if (firstRun){
            timer.setTimer(MIDDLEBALANCTIME2);
            //middleArmDelayTimer.setTimer(MIDDLEBALANCTIME3);
            firstRun = false;
        }

        /*if (middleArmDelayTimer.isExpired()){
            if (firstRunTimer){
                middleTimer.setTimer(MIDDLEBALANCTIME4);
            }

            PIDElbow.setPoint_Elbow = PIDElbow.DPAD_DOWN_ELBOW_COLLECT;
        }*/

        if (timer.isExpired()){
            if (firstRunTimer3_holy_cow_we_have_too_many_logic_variables_for_this_code_i_ned_to_fix){
                timer4_GOD_IGOTTASTOPADDINSOMANYTIMERS.setTimer(0.5);
                firstRunTimer3_holy_cow_we_have_too_many_logic_variables_for_this_code_i_ned_to_fix = false;

            }

            if (timer4_GOD_IGOTTASTOPADDINSOMANYTIMERS.isExpired()){  

                balance = true;
            } else {
                usingDrivetrainMotorsNOPOWER = true;
                Robot.isTeleop = true;
            }

            
            SmartDashboard.putBoolean("isTeleop", Robot.isTeleop);

        } else {
            if (forward) {
                usingDrivetrainMotorsForward = true;
            } else if(!forward){
                usingDrivetrainMotorsBackward = true;
            }

        }
    }


    public static void rightScoreMobility(){
        if (firstRun){
            timer.setTimer(RIGHTMOBILITYTIME);
            firstRun = false;
        }

        if (timer.isExpired()){
            TeleopDrivetrain.assignMotorPower(0,0);
            taskIsFinished = true;
        } else {
            TeleopDrivetrain.assignMotorPower(MOTORPOWER,-1*MOTORPOWER);
        }

        
    }
}

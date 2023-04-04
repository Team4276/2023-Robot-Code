package frc.auto;

import frc.utilities.RobotMode;
import frc.utilities.SoftwareTimer;
import frc.utilities.RobotMode.ROBOT_MODE;

public class BabyAuto {
    private static final double LEFTMOBILITYTIME = 4.5; // time in seconds
    private static final double MIDDLEBALANCTIME2 = 2.5; // drive up onto platform
    public static final double MOTORPOWER = 0.20;

    private static final double DEADZONE = 2;

    private static SoftwareTimer timer;
    private static SoftwareTimer timer4_GOD_IGOTTASTOPADDINSOMANYTIMERS;

    private static boolean firstRun = true;
    private static boolean firstRunTimer3_holy_cow_we_have_too_many_logic_variables_for_this_code_i_ned_to_fix = true;

    public static boolean taskIsFinished = false;

    public static AUTO_MOBILITY_MODE myAutoMobilityMode;

    public static enum AUTO_MOBILITY_MODE{
        FORWARD,
        BACKWARD,
        NOPOWER,
        TURNING
    }

    public static AUTO_MOBILITY_MODE get() {
        return myAutoMobilityMode;
    }
    
    public static void set(AUTO_MOBILITY_MODE val) {
        myAutoMobilityMode = val;
    }

    public static String getString() {

        switch (myAutoMobilityMode.ordinal()) {
          case 0:
            return "FORWARD";
          case 1:
            return "BACKWARD";
          case 2:
            return "NOPOWER";
          case 3:
            return "TURNING";
    
          default:
            break;
        }
        return "*****";
      }

    public static void BabyAutoInit(){
        timer = new SoftwareTimer();
        timer4_GOD_IGOTTASTOPADDINSOMANYTIMERS = new SoftwareTimer();

        myAutoMobilityMode = AUTO_MOBILITY_MODE.NOPOWER;
    }

    public static boolean ScoreMobility(){
        RobotMode.set(ROBOT_MODE.AUTO_DRIVING);

        if (firstRun){
            timer.setTimer(LEFTMOBILITYTIME);
            firstRun = false;
        }

        if (timer.isExpired()){
            set(AUTO_MOBILITY_MODE.NOPOWER);
            taskIsFinished = true;

            return true;
        } else {
            set(AUTO_MOBILITY_MODE.BACKWARD);

            return false;
        }
    }

    public static void middleBalance(boolean forward){
        RobotMode.set(ROBOT_MODE.AUTO_DRIVING);

        if (firstRun){
            timer.setTimer(MIDDLEBALANCTIME2);
            firstRun = false;
        }

        if (timer.isExpired()){
            if (firstRunTimer3_holy_cow_we_have_too_many_logic_variables_for_this_code_i_ned_to_fix){
                timer4_GOD_IGOTTASTOPADDINSOMANYTIMERS.setTimer(0.5);
                firstRunTimer3_holy_cow_we_have_too_many_logic_variables_for_this_code_i_ned_to_fix = false;

            }

            if (timer4_GOD_IGOTTASTOPADDINSOMANYTIMERS.isExpired()){  
                RobotMode.set(ROBOT_MODE.AUTO_BALANCING);


            } else {
                set(AUTO_MOBILITY_MODE.NOPOWER);
            }

        } else {
            if (forward) {
                set(AUTO_MOBILITY_MODE.FORWARD);
            } else if(!forward){
                set(AUTO_MOBILITY_MODE.BACKWARD);
            }

        }
        
    }

    public static void doabarrelroll(double current_angle, double desired_angle){
        RobotMode.set(ROBOT_MODE.AUTO_DRIVING);

        if (current_angle < desired_angle - DEADZONE){
            //turn
        } else if(current_angle > desired_angle + DEADZONE){
            //turn
        } else {
            //dont turn
        }
    }

    
}

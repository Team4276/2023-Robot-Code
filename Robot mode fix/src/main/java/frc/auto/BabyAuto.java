package frc.auto;

import frc.utilities.RobotMode;
import frc.utilities.SoftwareTimer;
import frc.utilities.RobotMode.ROBOT_MODE;

import edu.wpi.first.math.controller.PIDController;

public class BabyAuto {
    private static final double LEFTMOBILITYTIME = 4.5; // time in seconds
    private static final double MIDDLEBALANCTIME2 = 2.5; // drive up onto platform
    public static final double MOTORPOWER = 0.20;

    private static final double DEADZONE = 2;
    private static double desired_angle = 0;

    private static SoftwareTimer timer1;
    private static SoftwareTimer timer2;

    private static boolean firstRun1 = true;
    private static boolean firstRun2 = true;
    private static boolean firstRun3 = true;

    private static PIDController turnController;

    private static double kP = 1;
    private static double kI = 0;
    private static double kD = 0;

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
        timer1 = new SoftwareTimer();
        timer2 = new SoftwareTimer();

        myAutoMobilityMode = AUTO_MOBILITY_MODE.NOPOWER;

        
        turnController.setPID(kP, kI, kD);
    }

    public static boolean ScoreMobility(){
        RobotMode.set(ROBOT_MODE.AUTO_DRIVING);

        if (firstRun1){
            timer1.setTimer(LEFTMOBILITYTIME);
            firstRun1 = false;
        }

        if (timer1.isExpired()){
            set(AUTO_MOBILITY_MODE.NOPOWER);
            taskIsFinished = true;

            firstRun1 = true;

            return true;

        } else {
            set(AUTO_MOBILITY_MODE.BACKWARD);

            return false;
        }
    }

    public static void middleBalance(boolean forward){
        RobotMode.set(ROBOT_MODE.AUTO_DRIVING);

        if (firstRun1){
            timer1.setTimer(MIDDLEBALANCTIME2);
            firstRun1 = false;
        }

        if (timer1.isExpired()){
            if (firstRun2){
                timer2.setTimer(0.5);
                firstRun2 = false;

            }

            if (timer2.isExpired()){  
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

    public static double doabarrelroll(double current_angle){
        double power = 0;

        if (firstRun3){
            desired_angle = current_angle + 180;
            if (desired_angle > 360){
                desired_angle -= 360;
            }
            firstRun3 = false;
        }

        if(Math.abs(current_angle) > DEADZONE){
            power = turnController.calculate(current_angle, desired_angle);
        } else {
            firstRun3 = true;
        }

        return power;
    }
}

package frc.auto;

import frc.systems.Intake;
import frc.systems.PIDElbow;
import frc.systems.TeleopDrivetrain;
import frc.systems.Intake.intake;
import frc.systems.PIDElbow.Pos;
import frc.utilities.Gyroscope;
import frc.utilities.RobotMode;
import frc.utilities.SoftwareTimer;
import frc.utilities.RobotMode.ROBOT_MODE;

public class BabyAuto {
    private static final double LEFTMOBILITYTIME = 4.5; // time in seconds
    private static final double MIDDLEBALANCTIME2 = 2.5; // drive up onto platform
    public static final double MOTORPOWER = 0.20;

    private static final double FASTZONEPOWER = 0.2;
    private static final double SLOWZONE = 10;
    private static final double SLOWZONEPOWER = 0.1;
    private static final double DEADZONE = 2;
    private static double desired_angle = 0;

    private static SoftwareTimer timer1;
    private static SoftwareTimer timer2;
    private static SoftwareTimer timer3;
    private static SoftwareTimer timer4;

    private static boolean firstRun1 = true;
    private static boolean firstRun2 = true;
    private static boolean firstRun3 = true;
    private static boolean firstRun4 = true;
    private static boolean firstRun5 = true;
    private static boolean firstrun6 = true;


    public static boolean taskIsFinished = false;

    public static AUTO_MOBILITY_MODE myAutoMobilityMode;

    public static enum AUTO_MOBILITY_MODE{
        FORWARD,
        BACKWARD,
        NOPOWER,
        PICKUP
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
        timer3 = new SoftwareTimer();
        timer4 = new SoftwareTimer();

        myAutoMobilityMode = AUTO_MOBILITY_MODE.NOPOWER;

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

    public static boolean pickup(){
        if (firstRun3){
            desired_angle = Gyroscope.GetCorrectedYaw() + 180;
            firstRun3 = false;
        }

        if (Math.abs(desired_angle - Gyroscope.GetCorrectedYaw()) > SLOWZONE){
            TeleopDrivetrain.assignMotorPower(FASTZONEPOWER, FASTZONEPOWER);
            firstRun4 = true;
            return false;
        } else if (Math.abs(desired_angle - Gyroscope.GetCorrectedYaw()) > DEADZONE){
            firstRun4 = true;
            TeleopDrivetrain.assignMotorPower(SLOWZONEPOWER, SLOWZONEPOWER);
            return false;
        } else {
            TeleopDrivetrain.assignMotorPower(0, 0);
            if (firstRun4){
                if (collect()){
                    firstRun3 = true;
                    firstRun4 = false;
                }
                return false;

            } else {
                return true;
            }

        }


    }

    public static boolean collect(){
        if (firstRun5){
            timer3.setTimer(0.5);
        }
        
        if (timer3.isExpired()){
            if (firstrun6){
                timer4.setTimer(0.25);
            
            }

            if (timer4.isExpired()){
                Intake.intakeState = intake.OFF;
                PIDElbow.position = Pos.STOW;
            } else {
                Intake.intakeState = intake.INTAKE;
            }


            firstRun5 = true;
            return true;
        } else {
            PIDElbow.position = Pos.COLLECT;

            return false;
        }

    }
}

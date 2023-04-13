package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.utilities.Xbox;

public class PIDElbow {

    // Set points for DPAD -1.0 is in directopn of more extension
    private static double DPAD_RIGHT_ELBOW_REACH_NEAR_CONE = 0.0;
    private static double DPAD_UP_ELBOW_STOW = 0.0;
    private static double DPAD_LEFT_ELBOW_EJECT_CUBE = 0.0;
    private static double DPAD_DOWN_ELBOW_COLLECT = 0.0;

    private static CANSparkMax driveElbow;

    private static SparkMaxPIDController driveElbowPidController;

    private static RelativeEncoder driveElbowEncoder;

    private static SparkMaxLimitSwitch driveElbowLimitSwitch;

    private static double deadband = 0.2;

    // PID coefficients
    private static double[] kParray = {2e-2, 2e-2, 2e-2}; // 0: manual; 1: pos controlled; 2: hold
    private static double[] kIarray = {0, 0, 0};
    private static double[] kDarray = {0, 0, 0};
    private static double[] kIzarray = {0, 0, 0};
    private static double[] kFFarray = {0.000156, 0.000156, 0.000156};
    private static double kMaxOutputarray = 0.25;
    private static double kMinOutputarray = -0.25;
    private static double maxVelarray = 80;
    private static double maxAccarray = 10;
    private static double minVelarray = 0;
    private static double allowedErrarray = 0;

    private static double setPoint_Elbow = 0.0;

    private static double elbowZero = 0;

    public static Pos position = Pos.NONE;

    private static boolean firstRun = true;

    public enum Pos{
        NONE,
        MID_CONE,
        STOW,
        MID_CUBE,
        COLLECT,
        MANUAL,
        CALIBRATING,
        LIMIT,
        IDLE,

    }

    public static String getString(){

        switch(position.ordinal()){
            case 0:
              return "NONE";
            case 1:
              return "MID_CONE";
            case 2:
              return "STOW";
            case 3:
              return "MID_CUBE";
            case 4:
              return "COLLECT";
            case 5:
              return "MANUAL";
            case 6:
              return "CALIBRATING";
            case 7:
              return "LIMIT";
            case 8:
              return "IDLE";
      
            default:
              break;
          }
          return "*****";
        
    }

    public PIDElbow(int port) {
        driveElbow = new CANSparkMax(port, MotorType.kBrushless);

        driveElbowEncoder = driveElbow.getEncoder();

        setPoint_Elbow = driveElbowEncoder.getPosition();

        driveElbowEncoder.setPositionConversionFactor(1);

        driveElbowPidController = driveElbow.getPIDController();
        driveElbowPidController.setFeedbackDevice(driveElbowEncoder);

        driveElbowLimitSwitch = driveElbow.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        driveElbowLimitSwitch.enableLimitSwitch(true);

        int i = 2;

        while (i > -1){
            driveElbowPidController.setP(kParray[i],i);
            driveElbowPidController.setI(kIarray[i],i);
            driveElbowPidController.setD(kDarray[i],i);
            driveElbowPidController.setIZone(kIzarray[i],i);
            driveElbowPidController.setFF(kFFarray[i],i);
            driveElbowPidController.setOutputRange(kMaxOutputarray, kMinOutputarray, i);
            driveElbowPidController.setSmartMotionMaxVelocity(maxVelarray, i);
            driveElbowPidController.setSmartMotionMinOutputVelocity(minVelarray, i);
            driveElbowPidController.setSmartMotionMaxAccel(maxAccarray, i);
            driveElbowPidController.setSmartMotionAllowedClosedLoopError(allowedErrarray, i);
            i -= 1;
        }

    }

    private static void setPIDReference(double setPoint_Elbow, ControlType controlType, int slot) {
        driveElbowPidController.setReference(setPoint_Elbow, controlType, slot);
    }

    private static void setZero(){
        elbowZero = driveElbowEncoder.getPosition();

    }

    private static double getCorrectedPos(){
        return driveElbowEncoder.getPosition() - elbowZero;

    }

    public static void PIDElbowUpdate() {
        if (position == Pos.MANUAL){
            position = Pos.IDLE;
        }

        // ************************************************ \\
        // Command Inputs
        if (driveElbowLimitSwitch.isPressed()) {
            position = Pos.LIMIT;

        } else {
            if ((Math.abs(Robot.xboxController.getLeftY()) > deadband)) {
                position = Pos.MANUAL;
    
            } else if((Robot.xboxController.getRightTriggerAxis() > deadband) 
                && (Robot.xboxController.getLeftTriggerAxis() > deadband)){
                position = Pos.CALIBRATING;
    
            } else if (Robot.pov != -1) {
                if (Xbox.POVup == Robot.pov) {
                    position = Pos.STOW;
    
                } else if (Xbox.POVdown == Robot.pov) {
                    position = Pos.COLLECT;
    
                } else if (Xbox.POVright == Robot.pov) {
                    position = Pos.MID_CONE;
    
                } else if (Xbox.POVleft == Robot.pov) {
                    position = Pos.MID_CUBE;
    
                }
    
            }
        }
        

        // ************************************************ \\
        // Executing Commands
        if(Robot.xboxController.getAButton()){
            setZero();

        }
        
        if (position == Pos.STOW){
            setPoint_Elbow = DPAD_UP_ELBOW_STOW + elbowZero;
            setPIDReference(setPoint_Elbow, ControlType.kSmartMotion, 1);

        }
        
        if (position == Pos.COLLECT){
            setPoint_Elbow = DPAD_DOWN_ELBOW_COLLECT + elbowZero;
            setPIDReference(setPoint_Elbow, ControlType.kSmartMotion, 1);

        } 
        
        if (position == Pos.MID_CONE){
            setPoint_Elbow = DPAD_RIGHT_ELBOW_REACH_NEAR_CONE + elbowZero;
            setPIDReference(setPoint_Elbow, ControlType.kSmartMotion, 1);

        } 
        
        if (position == Pos.MID_CUBE){
            setPoint_Elbow = DPAD_LEFT_ELBOW_EJECT_CUBE + elbowZero;
            setPIDReference(setPoint_Elbow, ControlType.kSmartMotion, 1);

        }

        if (position == Pos.MANUAL){
            double speed = Robot.xboxController.getLeftY() * 12;
            setPIDReference(speed, ControlType.kSmartVelocity, 0);

            setPoint_Elbow = driveElbowEncoder.getPosition();

        }

        if (position == Pos.IDLE){
            setPIDReference(setPoint_Elbow, ControlType.kSmartMotion, 2);

        }

        
        if (position == Pos.LIMIT){
            if (Robot.xboxController.getRightY() > deadband){
                double speed = Robot.xboxController.getLeftY() * 12;
                setPIDReference(speed, ControlType.kSmartVelocity, 0);

                setPoint_Elbow = driveElbowEncoder.getPosition();

            } else {
                if (firstRun){
                    setPoint_Elbow = driveElbowEncoder.getPosition();
                    firstRun = false;
                }

                setPIDReference(setPoint_Elbow, ControlType.kSmartMotion, 1);
            }


        } else {
            firstRun = true;
        }

        if (position == Pos.CALIBRATING){
            if (Xbox.POVup == Robot.pov) {
                DPAD_UP_ELBOW_STOW = getCorrectedPos();

            } else if (Xbox.POVdown == Robot.pov) {
                DPAD_DOWN_ELBOW_COLLECT = getCorrectedPos();

            } else if (Xbox.POVright == Robot.pov) {
                DPAD_RIGHT_ELBOW_REACH_NEAR_CONE = getCorrectedPos();

            } else if (Xbox.POVleft == Robot.pov) {
                DPAD_LEFT_ELBOW_EJECT_CUBE = getCorrectedPos();

            }
        }
        

        SmartDashboard.putNumber("Raw Elbow Encoder:  ", driveElbowEncoder.getPosition());
        SmartDashboard.putNumber("Corrected Elbow Encoder", getCorrectedPos());
        SmartDashboard.putString("Current Position Being Calibrated", getString());
        SmartDashboard.putNumber("Elbow Power", driveElbow.getAppliedOutput());
        SmartDashboard.putNumber("DPAD_DOWN_ELBOW_COLLECT: ", DPAD_DOWN_ELBOW_COLLECT);
        SmartDashboard.putNumber("DPAD_LEFT_ELBOW_EJECT_CUBE: ", DPAD_LEFT_ELBOW_EJECT_CUBE);
        SmartDashboard.putNumber("DPAD_RIGHT_ELBOW_REACH_NEAR_CONE: ", DPAD_RIGHT_ELBOW_REACH_NEAR_CONE);
        SmartDashboard.putNumber("DPAD_UP_ELBOW_STOW: ", DPAD_UP_ELBOW_STOW);
        SmartDashboard.putNumber("Setpoint elbow", setPoint_Elbow);
    }

}


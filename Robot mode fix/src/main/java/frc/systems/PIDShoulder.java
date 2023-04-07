package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.utilities.Xbox;

public class PIDShoulder {

    // Set points for DPAD -1.0 is in directopn of more extension
    private static double DPAD_RIGHT_ELBOW_REACH_NEAR_CONE = -0.0;
    private static double DPAD_UP_ELBOW_STOW = -0.0;
    private static double DPAD_LEFT_ELBOW_EJECT_CUBE = -0.25;
    private static double DPAD_DOWN_ELBOW_COLLECT = -0.2;

    private static CANSparkMax driveShoulderR;
    private static CANSparkMax driveShoulderL;

    private static SparkMaxPIDController driveShoulderPidController;

    private static SparkMaxAbsoluteEncoder driveShoulderEncoder;

    private static double deadband = 0.2;

    private static double[] kParray = {3e-1, 5e-2, 3e-2};
    private static double[] kIarray = {0, 0, 0};
    private static double[] kDarray = {1e-1, 4e-1, 4e-1};
    private static double[] kIzarray = {0, 0, 0};
    private static double[] kFFarray = {0.000156, 0.000156, 0.000156};
    private static double[] kMaxOutputarray = {1, 1, 1};
    private static double[] kMinOutputarray = {-1, -1, -1};
    private static double[] maxVelarray = {75, 75, 75};
    private static double[] maxAccarray = {20, 20, 20};
    private static double[] minVelarray = {0.2, 0.2, 0.2};
    private static double[] allowedErrarray = {0, 0, 0};

    public static double setPoint_Shoulder;

    private static double shoulderZero = 0;

    private static int smartMotionSlot = 0;
    //0: manual
    //1: dpad
    //2: hold

    public static Pos position = Pos.NONE;

    private static final double SLOWZONE = 0.025;

    private enum Pos{
        NONE,
        MID_CONE,
        STOW,
        MID_CUBE,
        COLLECT,
        MANUAL,
        CALIBRATING

    }

    private static String getString(){

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
      
            default:
              break;
          }
          return "*****";
        
    }

    public PIDShoulder(int portR, int portL) {
        driveShoulderR = new CANSparkMax(portR, MotorType.kBrushless);
        driveShoulderL = new CANSparkMax(portL, MotorType.kBrushless);

        driveShoulderR.follow(driveShoulderL, true);
        
        driveShoulderEncoder = driveShoulderL.getAbsoluteEncoder(Type.kDutyCycle);

        setPoint_Shoulder = driveShoulderEncoder.getPosition();

        driveShoulderEncoder.setPositionConversionFactor(1);

        driveShoulderPidController = driveShoulderL.getPIDController();
        driveShoulderPidController.setFeedbackDevice(driveShoulderEncoder);

        CANSparkMax[] motorArray = { driveShoulderL };
        for (CANSparkMax motor : motorArray) {
            SparkMaxPIDController pidController = motor.getPIDController();
            
            int i = 2;

            while (i > -1){
                pidController.setP(kParray[i],i);
                pidController.setI(kIarray[i],i);
                pidController.setD(kDarray[i],i);
                pidController.setIZone(kIzarray[i],i);
                pidController.setFF(kFFarray[i],i);
                pidController.setOutputRange(kMaxOutputarray[i], kMinOutputarray[i], i);
                pidController.setSmartMotionMaxVelocity(maxVelarray[i], i);
                pidController.setSmartMotionMinOutputVelocity(minVelarray[i], i);
                pidController.setSmartMotionMaxAccel(maxAccarray[i], i);
                pidController.setSmartMotionAllowedClosedLoopError(allowedErrarray[i], i);
                i -= 1;
            }
        }
    }

    private static void setPIDReference(double setPoint_Shoulder, ControlType controlType, int smartMotionSlot) {
        driveShoulderPidController.setReference(setPoint_Shoulder, controlType, smartMotionSlot);
    }

    private static void setZero(){
        shoulderZero = driveShoulderEncoder.getPosition();

    }

    private static double getCorrectedPos(){
        return driveShoulderEncoder.getPosition() - shoulderZero;

    }

    public static void PIDShoulderUpdate() {
        // ************************************************ \\
        // Command Inputs
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

        } else {
            smartMotionSlot = 1;

            if (Math.abs(getCorrectedPos()) - setPoint_Shoulder < SLOWZONE){
                smartMotionSlot = 2;
            }

            setPIDReference(setPoint_Shoulder, ControlType.kSmartMotion, smartMotionSlot);

        }

        // ************************************************ \\
        // Executing Commands
        if(Robot.xboxController.getAButton()){
            setZero();

        }
        
        if (position == Pos.STOW){
            setPoint_Shoulder = DPAD_UP_ELBOW_STOW;

        }
        
        if (position == Pos.COLLECT){
            setPoint_Shoulder = DPAD_DOWN_ELBOW_COLLECT;

        } 
        
        if (position == Pos.MID_CONE){
            setPoint_Shoulder = DPAD_RIGHT_ELBOW_REACH_NEAR_CONE;

        } 
        
        if (position == Pos.MID_CUBE){
            setPoint_Shoulder = DPAD_LEFT_ELBOW_EJECT_CUBE;

        }

        if (position == Pos.MANUAL){
            double speed = Robot.xboxController.getLeftY() * 500;
            setPIDReference(speed, ControlType.kSmartVelocity, smartMotionSlot);

            setPoint_Shoulder = getCorrectedPos();

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
        

        SmartDashboard.putNumber("Raw Elbow Encoder:  ", driveShoulderEncoder.getPosition());
        SmartDashboard.putNumber("Corrected Elbow Encoder ", getCorrectedPos());
        SmartDashboard.putString("Current Position Being Calibrated ", getString());
        SmartDashboard.putNumber("Elbow Power R ", driveShoulderR.getAppliedOutput());
        SmartDashboard.putNumber("Elbow Power L ", driveShoulderL.getAppliedOutput());
        SmartDashboard.putNumber("DPAD_DOWN_ELBOW_COLLECT: ", DPAD_DOWN_ELBOW_COLLECT);
        SmartDashboard.putNumber("DPAD_LEFT_ELBOW_EJECT_CUBE: ", DPAD_LEFT_ELBOW_EJECT_CUBE);
        SmartDashboard.putNumber("DPAD_RIGHT_ELBOW_REACH_NEAR_CONE: ", DPAD_RIGHT_ELBOW_REACH_NEAR_CONE);
        SmartDashboard.putNumber("DPAD_UP_ELBOW_STOW: ", DPAD_UP_ELBOW_STOW);

    }

}

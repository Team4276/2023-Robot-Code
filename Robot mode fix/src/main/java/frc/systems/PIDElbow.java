package frc.systems;

import com.revrobotics.CANSparkMax;
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
    private static double DPAD_DOWN_ELBOW_COLLECT = 0.2;

    private static CANSparkMax driveElbow;

    private static SparkMaxPIDController driveElbowPidController;

    private static SparkMaxAbsoluteEncoder driveElbowEncoder;

    private static SparkMaxLimitSwitch driveElbowReverseLimitSwitch;

    private static double deadband = 0.2;

    // PID coefficients
    private static double kP = 2e-2;
    private static double kI = 0;
    private static double kD = 0;
    private static double kIz = 0;
    private static double kFF = 0.000156;

    private static double kMaxOutput = 1;
    private static double kMinOutput = -1;

    // Smart Motion Coefficients
    private static double maxVel = 80; // rpm
    private static double maxAcc = 10;
    private static double minVel = 0;

    private static double allowedErr = 0;

    private static double setPoint_Elbow = 0.0;

    private static double elbowZero = 0;

    public static Pos position = Pos.NONE;

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

    public PIDElbow(int port) {
        driveElbow = new CANSparkMax(port, MotorType.kBrushless);

        driveElbowReverseLimitSwitch = driveElbow.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        driveElbowReverseLimitSwitch.enableLimitSwitch(true);

        driveElbowEncoder = driveElbow.getAbsoluteEncoder(Type.kDutyCycle);

        setPoint_Elbow = driveElbowEncoder.getPosition();

        driveElbowEncoder.setPositionConversionFactor(1);

        driveElbowPidController = driveElbow.getPIDController();
        driveElbowPidController.setFeedbackDevice(driveElbowEncoder);

        int smartMotionSlot = 0;

        CANSparkMax[] motorArray = { driveElbow };
        for (CANSparkMax motor : motorArray) {
            SparkMaxPIDController pidController = motor.getPIDController();

            pidController.setP(kP);
            pidController.setI(kI);
            pidController.setD(kD);
            pidController.setIZone(kIz);
            pidController.setFF(kFF);
            pidController.setOutputRange(kMinOutput, kMaxOutput);
            pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
            pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
            pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
            pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        }

    }

    private static void setPIDReference(double setPoint_Elbow, ControlType controlType) {
        driveElbowPidController.setReference(setPoint_Elbow, controlType);
    }

    private static void setZero(){
        elbowZero = driveElbowEncoder.getPosition();

    }

    private static double getCorrectedPos(){
        return driveElbowEncoder.getPosition() - elbowZero;

    }

    public static void PIDElbowUpdate() {
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
            setPIDReference(setPoint_Elbow, ControlType.kSmartMotion);

        }

        // ************************************************ \\
        // Executing Commands
        if(Robot.xboxController.getAButton()){
            setZero();

        }
        
        if (position == Pos.STOW){
            setPoint_Elbow = DPAD_UP_ELBOW_STOW + elbowZero;

        }
        
        if (position == Pos.COLLECT){
            setPoint_Elbow = DPAD_DOWN_ELBOW_COLLECT + elbowZero;

        } 
        
        if (position == Pos.MID_CONE){
            setPoint_Elbow = DPAD_RIGHT_ELBOW_REACH_NEAR_CONE + elbowZero;

        } 
        
        if (position == Pos.MID_CUBE){
            setPoint_Elbow = DPAD_LEFT_ELBOW_EJECT_CUBE + elbowZero;

        }

        if (position == Pos.MANUAL){
            double speed = Robot.xboxController.getLeftY() * 500;
            setPIDReference(speed, ControlType.kSmartVelocity);

            setPoint_Elbow = driveElbowEncoder.getPosition();

        }

        if (position == Pos.CALIBRATING){
            if (Xbox.POVup == Robot.pov) {
                DPAD_RIGHT_ELBOW_REACH_NEAR_CONE = getCorrectedPos();

            } else if (Xbox.POVdown == Robot.pov) {
                DPAD_DOWN_ELBOW_COLLECT = getCorrectedPos();

            } else if (Xbox.POVright == Robot.pov) {
                DPAD_UP_ELBOW_STOW = getCorrectedPos();

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
    }

}

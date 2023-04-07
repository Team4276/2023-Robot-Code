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

    // PID coefficients
    private static double kP = 3e-1;
    private static double kI = 0;
    private static double kD = 1e-1;
    private static double kIz = 0;
    private static double kFF = 0.000156;

    private static double kPp = 5e-2;
    private static double kIp = 0;
    private static double kDp = 4e-1;
    private static double kIzp = 0;
    private static double kFFp = 0.000156;

    private static double kPph = 3e-2;
    private static double kIph = 0;
    private static double kDph = 4e-1;
    private static double kIzph = 0;
    private static double kFFph = 0.000156;

    private static double kMaxOutput = 1;
    private static double kMinOutput = -1;

    // Smart Motion Coefficients
    private static double maxVel = 75; // rpm
    private static double maxAcc = 20;
    private static double minVel = 0.2;

    private static double allowedErr = 0;

    public static double setPoint_Shoulder;

    private static double shoulderZero = 0;

    private static int smartMotionSlot = 0;

    private static final double SLOWZONE = 0.025;

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

            pidController.setP(kP, 0);
            pidController.setI(kI, 0);
            pidController.setD(kD, 0);
            pidController.setIZone(kIz, 0);
            pidController.setFF(kFF, 0);
            pidController.setOutputRange(kMinOutput, kMaxOutput, 0);
            pidController.setSmartMotionMaxVelocity(maxVel, 0);
            pidController.setSmartMotionMinOutputVelocity(minVel, 0);
            pidController.setSmartMotionMaxAccel(maxAcc, 0);
            pidController.setSmartMotionAllowedClosedLoopError(allowedErr, 0);

            pidController.setP(kPp, 1);
            pidController.setI(kIp, 1);
            pidController.setD(kDp, 1);
            pidController.setIZone(kIzp, 1);
            pidController.setFF(kFFp, 1);
            pidController.setOutputRange(kMinOutput, kMaxOutput, 1);
            pidController.setSmartMotionMaxVelocity(maxVel, 1);
            pidController.setSmartMotionMinOutputVelocity(minVel, 1);
            pidController.setSmartMotionMaxAccel(maxAcc, 1);
            pidController.setSmartMotionAllowedClosedLoopError(allowedErr, 1);

            pidController.setP(kPph, 2);
            pidController.setI(kIph, 2);
            pidController.setD(kDph, 2);
            pidController.setIZone(kIzph, 2);
            pidController.setFF(kFFph, 2);
            pidController.setOutputRange(kMinOutput, kMaxOutput, 2);
            pidController.setSmartMotionMaxVelocity(maxVel, 2);
            pidController.setSmartMotionMinOutputVelocity(minVel, 2);
            pidController.setSmartMotionMaxAccel(maxAcc, 2);
            pidController.setSmartMotionAllowedClosedLoopError(allowedErr, 2);

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
        if ((Math.abs(Robot.xboxController.getRightY()) > deadband)) {
            double speed = Robot.xboxController.getRightY();
            smartMotionSlot = 0;
            setPIDReference(speed, ControlType.kSmartVelocity, smartMotionSlot);

            setPoint_Shoulder = getCorrectedPos();

        } else if (Robot.pov != -1) {
            if (Xbox.POVup == Robot.pov) {
                setPoint_Shoulder = DPAD_UP_ELBOW_STOW;

            } else if (Xbox.POVdown == Robot.pov) {
                setPoint_Shoulder = DPAD_DOWN_ELBOW_COLLECT;

            } else if (Xbox.POVright == Robot.pov) {
                setPoint_Shoulder = DPAD_RIGHT_ELBOW_REACH_NEAR_CONE;

            } else if (Xbox.POVleft == Robot.pov) {
                setPoint_Shoulder = DPAD_LEFT_ELBOW_EJECT_CUBE;

            }

        } else if(Robot.xboxController.getAButton()){
            setZero();

        } else {
            smartMotionSlot = 1;

            if (Math.abs(getCorrectedPos() - setPoint_Shoulder) < SLOWZONE){
                smartMotionSlot = 2;
            }

            setPIDReference(setPoint_Shoulder, ControlType.kSmartMotion, smartMotionSlot);
        }


        SmartDashboard.putNumber("Raw Shoulder Encoder:  ", driveShoulderEncoder.getPosition());
        SmartDashboard.putNumber("Corrected Shoulder Encoder", getCorrectedPos());
        SmartDashboard.putNumber("Shoulder Power R", driveShoulderR.getAppliedOutput());
        SmartDashboard.putNumber("Shoulder Power L", driveShoulderL.getAppliedOutput());
        SmartDashboard.putNumber("SetPoint Shoulder", setPoint_Shoulder);

    }

}

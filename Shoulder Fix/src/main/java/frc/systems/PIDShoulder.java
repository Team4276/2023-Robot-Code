package frc.systems;

import com.revrobotics.AbsoluteEncoder;
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

public class PIDShoulder {

    // Set points for DPAD -1.0 is in directopn of more extension
    private static double DPAD_RIGHT_ELBOW_REACH_NEAR_CONE = 0.0;
    private static double DPAD_UP_ELBOW_STOW = 0.0;
    private static double DPAD_LEFT_ELBOW_EJECT_CUBE = 0.0;
    private static double DPAD_DOWN_ELBOW_COLLECT = 0.2;

    private static CANSparkMax driveShoulderR;
    private static CANSparkMax driveShoulderL;

    private static SparkMaxPIDController driveShoulderPidController;

    private static SparkMaxAbsoluteEncoder driveShoulderEncoder;

    private static SparkMaxLimitSwitch driveShoulderReverseLimitSwitch;

    private static double deadband = 0.2;

    // PID coefficients
    private static double kP = 0.0000001;
    private static double kI = 0;
    private static double kD = 0;
    private static double kIz = 0;
    private static double kFF = 0.000156;

    private static double kMaxOutput = 1;
    private static double kMinOutput = -1;

    // Smart Motion Coefficients
    private static double maxVel = 75; // rpm
    private static double maxAcc = 10;
    private static double minVel = 0;

    private static double allowedErr = 0;

    public static double setPoint_Shoulder;
    public static double test = 0;

    private static double shoulderZero = 0;

    public PIDShoulder(int portR, int portL) {
        driveShoulderR = new CANSparkMax(portR, MotorType.kBrushless);
        driveShoulderL = new CANSparkMax(portL, MotorType.kBrushless);

        driveShoulderR.restoreFactoryDefaults();
        driveShoulderL.restoreFactoryDefaults();

        driveShoulderR.follow(driveShoulderL, true);

        //driveShoulderReverseLimitSwitch = driveShoulderL.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        //driveShoulderReverseLimitSwitch.enableLimitSwitch(true);

        driveShoulderEncoder = driveShoulderL.getAbsoluteEncoder(Type.kDutyCycle);

        setPoint_Shoulder = driveShoulderEncoder.getPosition();

        driveShoulderEncoder.setPositionConversionFactor(1);

        driveShoulderPidController = driveShoulderL.getPIDController();
        driveShoulderPidController.setFeedbackDevice(driveShoulderEncoder);

        int smartMotionSlot = 0;

        CANSparkMax[] motorArray = { driveShoulderR };
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

    private static void setPIDReference(double setPoint_Shoulder, ControlType controlType) {
        driveShoulderPidController.setReference(setPoint_Shoulder, controlType);
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
            setPIDReference(speed, ControlType.kSmartVelocity);

            setPoint_Shoulder = driveShoulderEncoder.getPosition();

        } else if (Robot.pov != -1) {
            if (Xbox.POVup == Robot.pov) {
                setPoint_Shoulder = DPAD_UP_ELBOW_STOW + shoulderZero;

            } else if (Xbox.POVdown == Robot.pov) {
                setPoint_Shoulder = DPAD_DOWN_ELBOW_COLLECT + shoulderZero;

            } else if (Xbox.POVright == Robot.pov) {
                setPoint_Shoulder = DPAD_RIGHT_ELBOW_REACH_NEAR_CONE + shoulderZero;

            } else if (Xbox.POVleft == Robot.pov) {
                setPoint_Shoulder = DPAD_LEFT_ELBOW_EJECT_CUBE + shoulderZero;

            }

        } else if(Robot.xboxController.getAButton()){
            setZero();

        } else {
            setPIDReference(setPoint_Shoulder, ControlType.kSmartMotion);

        }


        SmartDashboard.putNumber("Raw Shoulder Encoder:  ", driveShoulderEncoder.getPosition());
        SmartDashboard.putNumber("Corrected Shoulder Encoder", getCorrectedPos());
        SmartDashboard.putNumber("Shoulder Power R", driveShoulderR.getAppliedOutput());
        SmartDashboard.putNumber("Shoulder Power L", driveShoulderL.getAppliedOutput());
        SmartDashboard.putNumber("SetPoint Shoulder", setPoint_Shoulder);

    }

}

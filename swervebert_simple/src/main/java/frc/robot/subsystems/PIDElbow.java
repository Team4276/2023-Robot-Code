package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElbowConstants;

public class PIDElbow {

    // Set points for DPAD -1.0 is in directopn of more extension
    public static double DPAD_UP_ELBOW_EJECT_BACK_MID = 0.484;
    public static double DPAD_LEFT_EJECT_FRONT_MID = 0.177;
    public static double DPAD_RIGHT_EJECT_FRONT_HIGH = 0.296;

    private static CANSparkMax driveElbow;

    private static SparkMaxPIDController driveElbowPidController;

    private static SparkMaxAbsoluteEncoder driveElbowEncoder;

    private static SparkMaxLimitSwitch driveElbowReverseLimitSwitch;
    private static SparkMaxLimitSwitch driveElbowForwardLimitSwitch;

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

    public static double setPoint_Elbow = 0.0;

    public static double elbowZero = 0.312;

    private static Pos position = Pos.NONE;

    private static boolean isJoystickActive = true;

    //TODO: fix everything

    private enum Pos {
        NONE,
        EJECT_FRONT_HIGH,
        EJECT_FRONT_MID,
        EJECT_BACK_MID,
    }

    private static String getString() {

        switch (position.ordinal()) {
            case 0:
                return "NONE";
            case 1:
                return "EJECT_FRONT_HIGH";
            case 2:
                return "EJECT_BACK_MID";

            default:
                break;
        }
        return "*****";
    }

    public PIDElbow() {
        driveElbow = new CANSparkMax(ElbowConstants.ElbowID, MotorType.kBrushless);

        driveElbowReverseLimitSwitch = driveElbow.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        driveElbowReverseLimitSwitch.enableLimitSwitch(true);
        driveElbowForwardLimitSwitch = driveElbow.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        driveElbowForwardLimitSwitch.enableLimitSwitch(true);

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

    public static void setZero() {
        elbowZero = driveElbowEncoder.getPosition();
    }

    private static double getCorrectedPos() {
        return driveElbowEncoder.getPosition() - elbowZero;
    }

    public void PIDElbowUpdate(double leftY, int POV) {
        if (leftY > deadband) {
            isJoystickActive = true;

            if (driveElbowForwardLimitSwitch.isPressed()) {
                // Joystick indicates move farther forward, but the forward limit switch is
                // pressed
                driveElbow.set(0);
            } else {
                double power = leftY / 3.75;
                driveElbow.set(power);
              }

        } else if (leftY < (-1*deadband)) {
            isJoystickActive = true;

            if (driveElbowReverseLimitSwitch.isPressed()) {
                // Joystick indicates move farther reverse, but the reverse limit switch is
                // pressed
                driveElbow.set(0);
            } else {
                double power = leftY / 3.75;
                driveElbow.set(power);
            }

        } else if (POV != 0) {
            if (POV == 1) {
                setPoint_Elbow = DPAD_UP_ELBOW_EJECT_BACK_MID + elbowZero;

            } else if (POV == 2) {
                setPoint_Elbow = DPAD_RIGHT_EJECT_FRONT_HIGH + elbowZero;

            } else if (POV == 4) {
                setPoint_Elbow = DPAD_LEFT_EJECT_FRONT_MID + elbowZero;

            } 
            setPIDReference(setPoint_Elbow, ControlType.kSmartMotion);

        } else if (Math.abs(leftY) <= deadband) {

            boolean isFirstTimeAfterJoystickReurnsToDeadband = isJoystickActive;
            isJoystickActive = false;
            if(isFirstTimeAfterJoystickReurnsToDeadband) {
                setPoint_Elbow = driveElbowEncoder.getPosition();
            }

            if ((driveElbowReverseLimitSwitch.isPressed())
                    || (driveElbowForwardLimitSwitch.isPressed())) {
                // At either limit we let gravity hold it
                driveElbow.set(0);
            } else {
                // When joystick command returnds to deadband, (and no DPAD command), hold
                // current position
                setPIDReference(setPoint_Elbow, ControlType.kSmartMotion);
            }
        }



        SmartDashboard.putNumber("Raw Elbow Encoder:  ", driveElbowEncoder.getPosition());
        SmartDashboard.putNumber("Corrected Elbow Encoder", getCorrectedPos());
        SmartDashboard.putString("Current Position Being Calibrated", getString());
        SmartDashboard.putNumber("Elbow Power", driveElbow.getAppliedOutput());
        SmartDashboard.putNumber("DPAD_UP_ELBOW_EJECT_BACK_MID: ", DPAD_UP_ELBOW_EJECT_BACK_MID);
        SmartDashboard.putNumber("DPAD_LEFT_EJECT_FRONT_MID: ", DPAD_LEFT_EJECT_FRONT_MID);
        SmartDashboard.putNumber("DPAD_RIGHT_EJECT_FRONT_HIGH: ", DPAD_RIGHT_EJECT_FRONT_HIGH);
        SmartDashboard.putBoolean("Intake Limit Switch", driveElbowReverseLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Stow Limit Switch", driveElbowForwardLimitSwitch.isPressed());
    }

}

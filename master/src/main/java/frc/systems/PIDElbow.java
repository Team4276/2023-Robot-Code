package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.utilities.RoboRioPorts;
import frc.utilities.Xbox;

public class PIDElbow {

    // Set points for DPAD
    public static final double DPAD_RIGHT_ELBOW_REACH_NEAR_CONE = -7.5;
    public static final double DPAD_UP_ELBOW_STOW = -1.0;
    public static final double DPAD_LEFT_ELBOW_EJECT_CUBE = -6;
    public static final double DPAD_DOWN_ELBOW_COLLECT = -14.5;

    private static CANSparkMax driveElbow;
    private static double deadband = 0.2;

    // PID coefficients
    private static double kP = 5e-5;
    private static double kI = 1e-6;
    private static double kD = 0;
    private static double kIz = 0;
    private static double kFF = 0.000156;
    private static double kMaxOutput = 1;
    private static double kMinOutput = -1;

    // Smart Motion Coefficients
    private static double maxVel = 2000; // rpm
    private static double maxAcc = 100;
    private static double minVel = 0;

    private static double allowedErr = 0;

    private static boolean modeIsSetPosition = false; // Otherwise set velocity

    private static double setPoint_Elbow = 0.0;
    private static DigitalInput limitSwitchElbow;
    private static double timeLastCalibration = 0.0;

    public PIDElbow(int port) {
        driveElbow = new CANSparkMax(port, MotorType.kBrushless);

        limitSwitchElbow = new DigitalInput(RoboRioPorts.DIO_LIMIT_ELBOW);
    }

    private static void setModePosition() {
        modeIsSetPosition = true;
    }

    private static void setModeVelocity() {
        modeIsSetPosition = false;
    }

    private static void setPIDReference(double setPoint_Elbow) {
        driveElbow.getPIDController().setReference(setPoint_Elbow,
                CANSparkMax.ControlType.kSmartMotion);
    }

    public static void PIDElbowInit() {
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

            calibrateElbowPosition();
            
            // Allow faster motion after calibration completes
            // TBD maxVel = 4000;
            // TBD maxAcc = 4000;            
        }

        setModePosition();
    }

    public static void calibrateElbowPosition() {
        double timeStart = 0.0;

        driveElbow.set(-1 * 0.2);
        Timer.delay(0.4); // Slowly extend for a short time, then normal update will pull it in until the
                          // limit switch closes

        timeStart = Timer.getFPGATimestamp();
        while (limitSwitchElbow.get()) {
            driveElbow.set(0.2);
            if ((Timer.getFPGATimestamp() - timeStart) > 4.0) {
                break;
            }
        }
        driveElbow.set(0.0);
        driveElbow.getEncoder().setPosition(0.0);
        setPoint_Elbow = DPAD_UP_ELBOW_STOW;
        timeLastCalibration = Timer.getFPGATimestamp();
        setModePosition();
    }

    public static void PIDElbowUpdate() {
        if (Robot.xboxController.getRawButton(Xbox.Y)) {
            if ((Timer.getFPGATimestamp() - timeLastCalibration) > 5.0) {
                calibrateElbowPosition();
            }
        }

        if ((Math.abs(Robot.xboxController.getLeftY()) > deadband)) {
            setModeVelocity();
        } else if (Robot.pov != -1) {
            setModePosition();
            if (Xbox.POVup == Robot.pov) {
                setPoint_Elbow = DPAD_UP_ELBOW_STOW;
            } else if (Xbox.POVdown == Robot.pov) {
                setPoint_Elbow = DPAD_DOWN_ELBOW_COLLECT;
            } else if (Xbox.POVright == Robot.pov) {
                setPoint_Elbow = DPAD_RIGHT_ELBOW_REACH_NEAR_CONE;
            } else if (Xbox.POVleft == Robot.pov) {
                setPoint_Elbow = DPAD_LEFT_ELBOW_EJECT_CUBE;
            }
        }

        if (modeIsSetPosition) {
            setPIDReference(setPoint_Elbow);

        } else if (Math.abs(Robot.xboxController.getLeftY()) < deadband) {
            driveElbow.getPIDController().setReference(0, CANSparkMax.ControlType.kVelocity);

        } else if (Math.abs(Robot.xboxController.getLeftY()) > deadband) {
            setPoint_Elbow = Robot.xboxController.getLeftY() * 500;
            driveElbow.getPIDController().setReference(setPoint_Elbow, CANSparkMax.ControlType.kVelocity);
        }

        if (!limitSwitchElbow.get()) {
            // Reset encoders all the time when the limit switch is in contact
            driveElbow.set(0.0);
            driveElbow.getEncoder().setPosition(0.0);
            setPoint_Elbow = DPAD_UP_ELBOW_STOW;
            setModePosition();
        }
    }

    // Speed inrange -1.0 to +1.0
    public static void setTestElbowSpeed(double speed) {
        if (Robot.isTestMode) {
            driveElbow.set(speed);
        }
    }
}

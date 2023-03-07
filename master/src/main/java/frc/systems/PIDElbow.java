package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.RoboRioPorts;
import frc.utilities.Xbox;

public class PIDElbow {

    // Set points for DPAD
    public static final int DPAD_UP_ELBOW_REACH_NEAR_CONE = 1;
    public static final int DPAD_DOWN_ELBOW_STOW = 1;
    public static final int DPAD_RIGHT_ELBOW_EJECT_CUBE = 1;
    public static final int DPAD_LEFT_ELBOW_COLLECT = 1;

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

    private static boolean usingSmartDashboard = false;
    private static boolean modeIsSetPosition = false; // Otherwise set velocity

    private static double setPoint_Elbow;

    private static final double NEAR_LIMIT_SWITCH_DISTANCE = 1.0;
    private static final double DEADZONE_LIMIT_SWITCH_DISTANCE = 0.05;

    private static DigitalInput limitSwitchElbow;

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
        SmartDashboard.putNumber("Elbow Set Position", setPoint_Elbow);
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
            pidController
                    .setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
        }

        if (usingSmartDashboard) {
            // display PID coefficients on SmartDashboard
            SmartDashboard.putNumber("Elbow P Gain", kP);
            SmartDashboard.putNumber("Elbow I Gain", kI);
            SmartDashboard.putNumber("Elbow D Gain", kD);
            SmartDashboard.putNumber("Elbow I Zone", kIz);
            SmartDashboard.putNumber("Elbow Feed Forward", kFF);
            SmartDashboard.putNumber("Elbow Max Output", kMaxOutput);
            SmartDashboard.putNumber("Elbow Min Output", kMinOutput);

            // display Smart Motion coefficients
            SmartDashboard.putNumber("Elbow Max Velocity", maxVel);
            SmartDashboard.putNumber("Elbow Min Velocity", minVel);
            SmartDashboard.putNumber("Elbow Max Acceleration", maxAcc);
            SmartDashboard.putNumber("Elbow Allowed Closed Loop Error", allowedErr);
            SmartDashboard.putNumber("Elbow Set Position", 0);
            SmartDashboard.putNumber("Elbow Set Velocity", 0);
        }

        setModePosition();
        setPoint_Elbow = DPAD_DOWN_ELBOW_STOW;
        calibrateElbowPosition();
    }

    public static void calibrateElbowPosition() {
        double timeStart = 0.0;

        driveElbow.set(-1 * 0.2);
        Timer.delay(0.4); // Slowly extend for a short time, then normal update will pull it in until the
                          // limit switch closes

        timeStart = Timer.getFPGATimestamp();
        while (limitSwitchElbow.get()) {
            driveElbow.set(-0.2);
            if( (Timer.getFPGATimestamp() - timeStart) > 1.0) {
                break;
            }
        }
        driveElbow.set(0.0);
        driveElbow.getEncoder().setPosition(0.0);
    }

    public static void PIDElbowUpdate() {
        if ((Math.abs(Robot.xboxController.getLeftY()) > deadband)) {
            setModeVelocity();
        } else if (Robot.pov != -1) {
            setModePosition();
            if (Xbox.POVup == Robot.pov) {
                setPoint_Elbow = DPAD_UP_ELBOW_REACH_NEAR_CONE;
            } else if (Xbox.POVdown == Robot.pov) {
                setPoint_Elbow = DPAD_DOWN_ELBOW_STOW;
            } else if (Xbox.POVright == Robot.pov) {
                setPoint_Elbow = DPAD_RIGHT_ELBOW_EJECT_CUBE;
            } else if (Xbox.POVleft == Robot.pov) {
                setPoint_Elbow = DPAD_LEFT_ELBOW_COLLECT;
            }
        }

        if (Robot.xboxController.getRawButton(Xbox.Y)) {
            calibrateElbowPosition();
        }

        if (modeIsSetPosition) {
            if (setPoint_Elbow != DPAD_DOWN_ELBOW_STOW) {
                setPIDReference(setPoint_Elbow);
            } else {
                if (!limitSwitchElbow.get()) {
                    driveElbow.set(0.0);
                } else  if (driveElbow.getEncoder().getPosition() < DEADZONE_LIMIT_SWITCH_DISTANCE) {
                        driveElbow.set(0.0);
                } else if (driveElbow.getEncoder().getPosition() < NEAR_LIMIT_SWITCH_DISTANCE) {
                    driveElbow.set(-1 * 0.2);
                } else {
                    driveElbow.set(-1 * 1.0);
                }
            }

        } else if (Math.abs(Robot.xboxController.getLeftY()) < deadband) {
            driveElbow.getPIDController().setReference(0, CANSparkMax.ControlType.kVelocity);

        } else if (Math.abs(Robot.xboxController.getLeftY()) > deadband) {
            setPoint_Elbow = Robot.xboxController.getLeftY() * 500;
            driveElbow.getPIDController().setReference(setPoint_Elbow, CANSparkMax.ControlType.kVelocity);
        }

        if (usingSmartDashboard) {
            Update(driveElbow);
        }
    }

    // Speed inrange -1.0 to +1.0
    public static void setTestElbowSpeed(double speed) {
        if (Robot.isTestMode) {
            driveElbow.set(speed);
        }
    }

    public static void updateTelemetry() {
        if (Robot.isTestMode) {
            if (Math.abs(Robot.xboxController.getRightY()) > deadband) {
                double rightY = Math.pow(Robot.xboxController.getRightY(), 3 / 2);
                setTestElbowSpeed(rightY);
            } else {
                setTestElbowSpeed(0);
            }
        } else {

            SmartDashboard.putNumber("Elbow Encoder_Pos", driveElbow.getEncoder().getPosition());
            SmartDashboard.putNumber("Elbow Encoder_Vel", driveElbow.getEncoder().getVelocity());
            if (modeIsSetPosition) {
                SmartDashboard.putString("Elbow Mode", " Position");
                SmartDashboard.putNumber("Elbow setPoint_Elbow_Pos", setPoint_Elbow);
                SmartDashboard.putNumber("Elbow MotorOutput_Pos", driveElbow.getAppliedOutput());
            } else {
                SmartDashboard.putString("Elbow Mode", " Velocity");
            }

        }
    }

    private static void Update(CANSparkMax motor) {
        SparkMaxPIDController pidController = motor.getPIDController();

        double p = SmartDashboard.getNumber("Elbow P Gain", 0);
        double i = SmartDashboard.getNumber("Elbow I Gain", 0);
        double d = SmartDashboard.getNumber("Elbow D Gain", 0);
        double iz = SmartDashboard.getNumber("Elbow I Zone", 0);
        double ff = SmartDashboard.getNumber("Elbow Feed Forward", 0);
        double max = SmartDashboard.getNumber("Elbow Max Output", 0);
        double min = SmartDashboard.getNumber("Elbow Min Output", 0);
        double maxV = SmartDashboard.getNumber("Elbow Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Elbow Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Elbow Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Elbow Allowed Closed Loop Error", 0);

        if ((p != kP)) {
            pidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            pidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            pidController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            pidController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            pidController.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }
        if ((maxV != maxVel)) {
            pidController.setSmartMotionMaxVelocity(maxV, 0);
            maxVel = maxV;
        }
        if ((minV != minVel)) {
            pidController.setSmartMotionMinOutputVelocity(minV, 0);
            minVel = minV;
        }
        if ((maxA != maxAcc)) {
            pidController.setSmartMotionMaxAccel(maxA, 0);
            maxAcc = maxA;
        }
        if ((allE != allowedErr)) {
            pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
            allowedErr = allE;
        }
    }
}

package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.utilities.RoboRioPorts;
import frc.utilities.SoftwareTimer;
import frc.utilities.Xbox;

public class PIDElbow {

    // Set points for DPAD -1.0 is in directopn of more extension
    public static final double DPAD_RIGHT_ELBOW_REACH_NEAR_CONE = 7.5;
    public static final double DPAD_UP_ELBOW_STOW = 0.0;
    public static final double DPAD_LEFT_ELBOW_EJECT_CUBE = 3;
    public static final double DPAD_DOWN_ELBOW_COLLECT = 8.25;

    private static final double NEAR_CONE_DELAY = 0;
    private static final double STOW_DELAY = 0;
    private static final double CUBE_DELAY = 0;
    private static final double COLLECT_DELAY = 0;

    private static SoftwareTimer delayTimer;
    private static boolean firstRun = true;

    private static CANSparkMax driveElbow;

    private static SparkMaxPIDController driveElbowPidController;

    private static RelativeEncoder driveElbowEncoder;
    private static final int CPR = 42;

    private static double deadband = 0.2;

    // PID coefficients
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static double kIz = 0;
    private static double kFF = 0.000156;
    private static double kMaxOutput = 1;
    private static double kMinOutput = -1;

    // Smart Motion Coefficients
    private static double maxVel = 40; // rpm
    private static double maxAcc = 10;
    private static double minVel = 0;

    private static double allowedErr = 0;

    private static boolean modeIsSetPosition = false; // Otherwise set velocity

    public static double setPoint_Elbow = 0.0;
    private static DigitalInput limitSwitchElbow;

    public PIDElbow(int port) {
        driveElbow = new CANSparkMax(port, MotorType.kBrushless);

        driveElbowEncoder = driveElbow.getEncoder();

        driveElbowPidController = driveElbow.getPIDController();

        limitSwitchElbow = new DigitalInput(RoboRioPorts.DIO_LIMIT_ELBOW);

        delayTimer = new SoftwareTimer();
    }

    private static void setModePosition() {
        modeIsSetPosition = true;
    }

    private static void setModeVelocity() {
        modeIsSetPosition = false;
    }

    private static void setPIDReference(double setPoint_Elbow) {
        driveElbowPidController.setReference(setPoint_Elbow,
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

        }

        setModePosition();
    }

    public static void PIDElbowUpdate() {
        if ((Math.abs(Robot.xboxController.getLeftY()) > deadband)) {
            setModeVelocity();
        } else if (Robot.pov != -1) {
            setModePosition();


            if (Xbox.POVup == Robot.pov) {
                if(firstRun){
                    delayTimer.setTimer(STOW_DELAY);
                    firstRun = false;
                }

                if (delayTimer.isExpired()){
                    setPoint_Elbow = DPAD_UP_ELBOW_STOW;
                    firstRun = true;
                }
            } else if (Xbox.POVdown == Robot.pov) {
                if(firstRun){
                    delayTimer.setTimer(COLLECT_DELAY);
                    firstRun = false;
                }

                if (delayTimer.isExpired()){
                    setPoint_Elbow = DPAD_DOWN_ELBOW_COLLECT;
                    firstRun = true;
                }

            } else if (Xbox.POVright == Robot.pov) {
                if(firstRun){
                    delayTimer.setTimer(NEAR_CONE_DELAY);
                    firstRun = false;
                }

                if (delayTimer.isExpired()){
                    setPoint_Elbow = DPAD_RIGHT_ELBOW_REACH_NEAR_CONE;
                    firstRun = true;
                }
                
            } else if (Xbox.POVleft == Robot.pov) {
                if(firstRun){
                    delayTimer.setTimer(CUBE_DELAY);
                    firstRun = false;
                }

                if (delayTimer.isExpired()){
                    setPoint_Elbow = DPAD_LEFT_ELBOW_EJECT_CUBE;
                    firstRun = true;
                }
            }
        }

        if (modeIsSetPosition) {
            setPIDReference(setPoint_Elbow);

        } else if (Math.abs(Robot.xboxController.getLeftY()) < deadband) {
            driveElbow.getPIDController().setReference(0, CANSparkMax.ControlType.kVelocity);

        } else if (Math.abs(Robot.xboxController.getLeftY()) > deadband) {
            setPoint_Elbow = (-1*Robot.xboxController.getLeftY()) * 1000;
            driveElbow.getPIDController().setReference(setPoint_Elbow, CANSparkMax.ControlType.kVelocity);

        }

        if (!limitSwitchElbow.get()) {
            if (Math.abs(driveElbow.getEncoder().getPosition()) > 0.05) {
                // Reset encoders all the time when the limit switch is in contact
                driveElbowEncoder.setPosition(0);
            }
        }

        SmartDashboard.putNumber("ElbowEncoder:  ", driveElbowEncoder.getPosition());
        SmartDashboard.putNumber("Elbow Power", driveElbow.getAppliedOutput());
    }

    // Speed inrange -1.0 to +1.0
    public static void setTestElbowSpeed(double speed) {
        if (Robot.isTestMode) {
            driveElbow.set(speed);
        }
    }
}

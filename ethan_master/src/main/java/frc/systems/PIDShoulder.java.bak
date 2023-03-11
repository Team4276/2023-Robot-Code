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

public class PIDShoulder {

    // Set points for DPAD
    public static final int DPAD_RIGHT_SHOULDER_REACH_NEAR_CONE = 12;
    public static final int DPAD_UP_SHOULDER_STOW = 0;
    public static final int DPAD_LEFT_SHOULDER_EJECT_CUBE = 3;
    public static final int DPAD_DOWN_ULDER_COLLECT = 0;

    public static CANSparkMax driveShoulder_R;
    public static CANSparkMax driveShoulder_L;

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
    private static double deadband = 0.2;

    private static double allowedErr = 0;

    public static double setPoint_Shoulder = 0.0;

    private static DigitalInput limitSwitchShoulder;
    private static double timeLastCalibration = 0.0;

    public PIDShoulder(int port_R, int port_L) {
        driveShoulder_R = new CANSparkMax(port_R, MotorType.kBrushless);
        driveShoulder_L = new CANSparkMax(port_L, MotorType.kBrushless);

        limitSwitchShoulder = new DigitalInput(RoboRioPorts.DIO_LIMIT_SHOULDER);
    }

    private static void setPIDReference(double setPoint_Shoulder) {
        driveShoulder_R.getPIDController().setReference(setPoint_Shoulder, CANSparkMax.ControlType.kSmartMotion);
        driveShoulder_L.getPIDController().setReference(-1 * setPoint_Shoulder, CANSparkMax.ControlType.kSmartMotion); // -1
    }

    private static void setShoulderSpeed(double speed) {
        driveShoulder_R.set(speed);
        driveShoulder_L.set(-1 * speed);
    }

    public static void PIDShoulderInit() {
        int smartMotionSlot = 0;

        CANSparkMax[] motorArray = { driveShoulder_R, driveShoulder_L };
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

            calibrateShoulderPosition();
             
            // Allow faster motion after calibration completes
            maxVel = 3000;
            maxAcc = 4000;            
        }
    }

    public static void calibrateShoulderPosition() {
        double timeStart = 0.0;

        setShoulderSpeed(-1 * 0.2);
        Timer.delay(0.4); // Slowly extend for a short time, then normal update will pull it in until the
                          // limit switch closes

        timeStart = Timer.getFPGATimestamp();
        while (limitSwitchShoulder.get()) {
            setShoulderSpeed(0.2);
            if ((Timer.getFPGATimestamp() - timeStart) > 4.0) {
                break;
            }
        }
        setShoulderSpeed(0.0);
        driveShoulder_R.getEncoder().setPosition(0.0);
        driveShoulder_L.getEncoder().setPosition(0.0);
        setPoint_Shoulder = DPAD_UP_SHOULDER_STOW;
        timeLastCalibration = Timer.getFPGATimestamp();
    }

    public static void PIDShoulderUpdate() {

        if (Robot.xboxController.getRawButton(Xbox.A)) {
            if ((Timer.getFPGATimestamp() - timeLastCalibration) > 5.0) {
                calibrateShoulderPosition();
            }
        }

        if (Robot.pov != -1) {
            if (Xbox.POVup == Robot.pov) {
                setPoint_Shoulder = DPAD_UP_SHOULDER_STOW;
            } else if (Xbox.POVdown == Robot.pov) {
                setPoint_Shoulder = DPAD_DOWN_ULDER_COLLECT;
            } else if (Xbox.POVright == Robot.pov) {
                setPoint_Shoulder = DPAD_RIGHT_SHOULDER_REACH_NEAR_CONE;
            } else if (Xbox.POVleft == Robot.pov) {
                setPoint_Shoulder = DPAD_LEFT_SHOULDER_EJECT_CUBE;
            }
        }
        
        //setPIDReference(setPoint_Shoulder);

        if (Math.abs(Robot.xboxController.getRightY()) < deadband) {
            driveShoulder_R.getPIDController().setReference(0, CANSparkMax.ControlType.kVelocity);
            driveShoulder_L.getPIDController().setReference(0, CANSparkMax.ControlType.kVelocity);

        } else if (Math.abs(Robot.xboxController.getRightY()) > deadband) {
            setPoint_Shoulder = Robot.xboxController.getRightY() * 500;
            driveShoulder_R.getPIDController().setReference(-1 * setPoint_Shoulder, CANSparkMax.ControlType.kVelocity);
            driveShoulder_L.getPIDController().setReference(setPoint_Shoulder, CANSparkMax.ControlType.kVelocity);
        }

        

        if (!limitSwitchShoulder.get()) {
            if(Math.abs(driveShoulder_R.getEncoder().getPosition()) > 0.05) {
                // Reset encoders all the time when the limit switch is in contact
                 driveShoulder_R.getEncoder().setPosition(0.0);
            }
            if(Math.abs(driveShoulder_L.getEncoder().getPosition()) > 0.05) {
                // Reset encoders all the time when the limit switch is in contact
                 driveShoulder_L.getEncoder().setPosition(0.0);
            }
        }
                
        SmartDashboard.putNumber("ShoulderEncoder_R:  ", driveShoulder_R.getEncoder().getPosition());
        SmartDashboard.putNumber("ShoulderEncoder_L:  ", driveShoulder_L.getEncoder().getPosition());
    }
}

package frc.systems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
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

    public static AbsoluteEncoder driveShoulder_R_Encoder;
    public static AbsoluteEncoder driveShoulder_L_Encoder;

    // PID coefficients
    private static double kP = 5e-3;
    private static double kI = 0;
    private static double kD = 5e-3;
    private static double kIz = 0;
    private static double kFF = 0.000156;
    private static double kMaxOutput = 1;
    private static double kMinOutput = -1;

    // Smart Motion Coefficients
    private static double maxVel = 75; // rpm
    private static double maxAcc = 10;
    private static double minVel = 0;

    private static double allowedErr = 0;

    public static double setPoint_Shoulder = 0.0;

    private static DigitalInput limitSwitchShoulder;

    private static double deadband = 0.2;

    public PIDShoulder(int port_R, int port_L) {
        driveShoulder_R = new CANSparkMax(port_R, MotorType.kBrushless);
        driveShoulder_L = new CANSparkMax(port_L, MotorType.kBrushless);

        driveShoulder_L.follow(driveShoulder_R, true);

        limitSwitchShoulder = new DigitalInput(RoboRioPorts.DIO_LIMIT_SHOULDER);
    }

    private static void setPIDReference(double setPoint_Shoulder) {
        driveShoulder_R.getPIDController().setReference(setPoint_Shoulder, CANSparkMax.ControlType.kSmartMotion);
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

        }
    }

    public static void PIDShoulderUpdate() {

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

        if (Math.abs(Robot.xboxController.getRightY()) < deadband) {
            setPIDReference(setPoint_Shoulder);
        } else if (Math.abs(Robot.xboxController.getRightY()) > deadband) {
            setPoint_Shoulder = 0;
            double power = Robot.xboxController.getRightY() / 5;

            driveShoulder_R.set(power);

        }

        if (!limitSwitchShoulder.get()) {
            if (Math.abs(driveShoulder_R.getEncoder().getPosition()) > 0.05) {
                // Reset encoders all the time when the limit switch is in contact
                driveShoulder_R.getEncoder().setPosition(0.0);
            }
            if (Math.abs(driveShoulder_L.getEncoder().getPosition()) > 0.05) {
                // Reset encoders all the time when the limit switch is in contact
                driveShoulder_L.getEncoder().setPosition(0.0);
            }
        }

        SmartDashboard.putNumber("ShoulderEncoder_R:  ", driveShoulder_R.getEncoder().getPosition());
        SmartDashboard.putNumber("ShoulderEncoder_L:  ", driveShoulder_L.getEncoder().getPosition());
    }
}

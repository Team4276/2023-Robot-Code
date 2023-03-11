package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.PIDMotor;
import frc.utilities.RoboRioPorts;
import frc.utilities.Xbox;

public class NewPIDShoulder {

    // Set points for DPAD
    public static final int DPAD_RIGHT_SHOULDER_REACH_NEAR_CONE = 12;
    public static final int DPAD_UP_SHOULDER_STOW = 0;
    public static final int DPAD_LEFT_SHOULDER_EJECT_CUBE = 3;
    public static final int DPAD_DOWN_ULDER_COLLECT = 0;

    public static CANSparkMax driveShoulder_R;
    public static CANSparkMax driveShoulder_L;

    // PID coefficients
    private static double kP = 1e-4;
    private static double kI = 0;
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

    public static double setPoint_Shoulder = 0.0;

    private static DigitalInput limitSwitchShoulder;
    private static double timeLastCalibration = 0.0;

    private static RelativeEncoder Shoulder_Encoder_R;
    private static RelativeEncoder Shoulder_Encoder_L;

    public NewPIDShoulder(int port_R, int port_L) {
        driveShoulder_R = new CANSparkMax(port_R, MotorType.kBrushless);
        driveShoulder_L = new CANSparkMax(port_L, MotorType.kBrushless);
        driveShoulder_L.follow(driveShoulder_R);
        driveShoulder_L.setInverted(true);

        Shoulder_Encoder_R = driveShoulder_R.getEncoder(Type.kHallSensor, 42);
        Shoulder_Encoder_L = driveShoulder_L.getEncoder(Type.kHallSensor, 42);

        // Shoulder_Encoder_R = driveShoulder_R.getEncoder();
        // Shoulder_Encoder_L = driveShoulder_L.getEncoder();
        

        limitSwitchShoulder = new DigitalInput(RoboRioPorts.DIO_LIMIT_SHOULDER);
        driveShoulder_L.burnFlash();
        driveShoulder_R.burnFlash();
    }

    private static void setPIDReference(double setPoint_Shoulder) {
        double vel = PIDMotor.getOutput(Shoulder_Encoder_R.getPosition(), setPoint_Shoulder);

        driveShoulder_R.set(vel);

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
        Shoulder_Encoder_R.setPosition(0);
        Shoulder_Encoder_L.setPosition(0);
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
        
        setPIDReference(setPoint_Shoulder);

        

        if (!limitSwitchShoulder.get()) {
            if(Math.abs(Shoulder_Encoder_R.getPosition()) < 0.05) {
                // Reset encoders all the time when the limit switch is in contact
                Shoulder_Encoder_R.setPosition(0);
            }
            if(Math.abs(Shoulder_Encoder_L.getPosition()) < 0.05) {
                // Reset encoders all the time when the limit switch is in contact
                Shoulder_Encoder_L.setPosition(0);
            }
        }
                
        SmartDashboard.putNumber("ShoulderEncoder_R:  ", Shoulder_Encoder_R.getPosition());
        SmartDashboard.putNumber("ShoulderEncoder_L:  ", Shoulder_Encoder_L.getPosition());
        SmartDashboard.putNumber("L motor output", driveShoulder_L.getAppliedOutput());
        SmartDashboard.putNumber("Setpoint Shoulder", setPoint_Shoulder);
    }
}

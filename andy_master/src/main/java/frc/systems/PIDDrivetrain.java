package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.utilities.RobotMode;
import frc.utilities.RobotMode.ROBOT_MODE;

public class PIDDrivetrain extends BaseDrivetrain {
    public PIDDrivetrain(int FLport, int BLport, int FRport, int BRport) {

        super(FLport, BLport, FRport, BRport);
        // TODO Auto-generated constructor stub
    }

    // PID coefficients
    private static double kP = 5e-5;
    private static double kI = 1e-6;
    private static double kD = 0;
    private static double kIz = 0;
    private static double kFF = 0.000156;
    private static double kMaxOutput = 1;
    private static double kMinOutput = -1;

    // Smart Motion Coefficients
    private static double maxVel = 1000; // rpm
    private static double maxAcc = 100;
    private static double minVel = 0;

    private static double allowedErr = 0;

    private static double holdThisPosition = 0;

    public static boolean newPositiontohold = true;

    public static double setPoint;

    private static double leftRPM = 0;
    private static double rightRPM = 0;

    public static void PIDDrivetrainInit() {
        int smartMotionSlot = 0;

        CANSparkMax[] motorArray = { frDriveX, flDriveX };
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

    public static void PIDDrivetrainUpdate() {
        Update(1, frDriveX, false, false);
        Update(-1, flDriveX, true, true);
    }

    public static void Update(double sign, CANSparkMax motor, boolean lastMotor, boolean left) {
        SparkMaxPIDController pidController = motor.getPIDController();
        RelativeEncoder encoder = motor.getEncoder();

        if (RobotMode.get() == ROBOT_MODE.HOLD_POSITION) {
            if (newPositiontohold) {
                holdThisPosition = encoder.getPosition();
                if (lastMotor){
                    newPositiontohold = false;
                }

                pidController.setReference(holdThisPosition, CANSparkMax.ControlType.kSmartMotion);
            }


        } else if (RobotMode.get() == ROBOT_MODE.BALANCING) {
            newPositiontohold = true;
            pidController.setReference(setPoint * sign, CANSparkMax.ControlType.kVelocity);
        } else if ((RobotMode.get() == ROBOT_MODE.TELEOP_APPROACHING_PATH_START) || (RobotMode.get() == ROBOT_MODE.TELEOP_FOLLOWING_PATH)) {
            newPositiontohold = true;

            if (left){
                pidController.setReference(leftRPM, ControlType.kVelocity);
            } else {
                pidController.setReference(rightRPM, ControlType.kVelocity);
            }
        }
    }

    public static void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        leftRPM = speeds.leftMetersPerSecond / 0.05727192;
        rightRPM = speeds.leftMetersPerSecond / 0.05727192;

    }

}
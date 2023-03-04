package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.Xbox;

public class PIDShoulder {
    
    // Set points for DPAD
    public static final int DPAD_UP_SHOULDER_REACH_NEAR_CONE = 10;
    public static final int DPAD_DOWN_SHOULDER_STOW = 7;
    public static final int DPAD_RIGHT_SHOULDER_EJECT_CUBE = 14;
    public static final int DPAD_LEFT_SHOULDER_COLLECT = 13;

    private static CANSparkMax driveShoulder_R;
    private static CANSparkMax driveShoulder_L;

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

    private static boolean usingSmartDashboard = false;

    private static double setPoint_Shoulder;

    public PIDShoulder(int port_R, int port_L) {
        driveShoulder_R = new CANSparkMax(port_R, MotorType.kBrushless);
        driveShoulder_L = new CANSparkMax(port_L, MotorType.kBrushless);
    }

    private static void setPIDReference(double setPoint_Shoulder) {
        SmartDashboard.putNumber("Shoulder Set Position", setPoint_Shoulder);
        driveShoulder_R.getPIDController().setReference(setPoint_Shoulder, CANSparkMax.ControlType.kSmartMotion);
        driveShoulder_L.getPIDController().setReference(-1 * setPoint_Shoulder, CANSparkMax.ControlType.kSmartMotion); // -1 because the left motor turns the opposite way when the arm moves
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

        if (usingSmartDashboard) {
            // display PID coefficients on SmartDashboard
            SmartDashboard.putNumber("Shoulder P Gain", kP);
            SmartDashboard.putNumber("Shoulder I Gain", kI);
            SmartDashboard.putNumber("Shoulder D Gain", kD);
            SmartDashboard.putNumber("Shoulder I Zone", kIz);
            SmartDashboard.putNumber("Shoulder Feed Forward", kFF);
            SmartDashboard.putNumber("Shoulder Max Output", kMaxOutput);
            SmartDashboard.putNumber("Shoulder Min Output", kMinOutput);

            // display Smart Motion coefficients
            SmartDashboard.putNumber("Shoulder Max Velocity", maxVel);
            SmartDashboard.putNumber("Shoulder Min Velocity", minVel);
            SmartDashboard.putNumber("Shoulder Max Acceleration", maxAcc);
            SmartDashboard.putNumber("Shoulder Allowed Closed Loop Error", allowedErr);
            SmartDashboard.putNumber("Shoulder Set Position", 0);
            SmartDashboard.putNumber("Shoulder Set Velocity", 0);
        }
    }

    public static void PIDShoulderUpdate() {
        if (Robot.pov != -1) {

            if (Xbox.POVup == Robot.pov) {
                setPoint_Shoulder = DPAD_UP_SHOULDER_REACH_NEAR_CONE;
                setPIDReference(setPoint_Shoulder);
            } else if (Xbox.POVdown == Robot.pov) {
                setPoint_Shoulder = DPAD_DOWN_SHOULDER_STOW;
                setPIDReference(setPoint_Shoulder);
            } else if (Xbox.POVright == Robot.pov) {
                setPoint_Shoulder = DPAD_RIGHT_SHOULDER_EJECT_CUBE;
                setPIDReference(setPoint_Shoulder);
            } else if (Xbox.POVleft == Robot.pov) {
                setPoint_Shoulder = DPAD_LEFT_SHOULDER_COLLECT;
                setPIDReference(setPoint_Shoulder);
            }
        }

        if (usingSmartDashboard) {
            Update(driveShoulder_R);
            Update(driveShoulder_L);
        }
    }

    public static void updateTelemetry() {
        //Shoulder Setpoints
        SmartDashboard.putNumber("SetPoint_SR_Pos", setPoint_Shoulder);
        SmartDashboard.putNumber("SetPoint_SL_Pos", setPoint_Shoulder);

        //Encoder Positions
        SmartDashboard.putNumber("Encoder_SR_Pos", driveShoulder_R.getEncoder().getPosition());
        SmartDashboard.putNumber("Encoder_SL_Pos", driveShoulder_L.getEncoder().getPosition());

        //Shoulder Motor Outputs
        SmartDashboard.putNumber("MotorOutput_SR_Pos", driveShoulder_R.getAppliedOutput());
        SmartDashboard.putNumber("MotorOutput_SL_Pos", driveShoulder_L.getAppliedOutput());
    }

    private static void Update(CANSparkMax motor) {
        SparkMaxPIDController pidController = motor.getPIDController();

        double p = SmartDashboard.getNumber("Shoulder P Gain", 0);
        double i = SmartDashboard.getNumber("Shoulder I Gain", 0);
        double d = SmartDashboard.getNumber("Shoulder D Gain", 0);
        double iz = SmartDashboard.getNumber("Shoulder I Zone", 0);
        double ff = SmartDashboard.getNumber("Shoulder Feed Forward", 0);
        double max = SmartDashboard.getNumber("Shoulder Max Output", 0);
        double min = SmartDashboard.getNumber("Shoulder Min Output", 0);
        double maxV = SmartDashboard.getNumber("Shoulder Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Shoulder Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Shoulder Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Shoulder Allowed Closed Loop Error", 0);

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

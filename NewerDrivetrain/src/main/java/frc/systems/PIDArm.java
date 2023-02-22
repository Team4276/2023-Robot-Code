package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class PIDArm {
    private static CANSparkMax driveShoulder_R;
    private static CANSparkMax driveShoulder_L;
    private static CANSparkMax driveWrist;

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
    private static double deadband = 0.05;

    private static double multiplier = 500;

    public PIDArm(int SRport, int SLport, int Wportt) {
        driveShoulder_R = new CANSparkMax(SRport, MotorType.kBrushless);
        driveShoulder_L = new CANSparkMax(SLport, MotorType.kBrushless);
        driveWrist = new CANSparkMax(Wportt, MotorType.kBrushless);
    }

    public static boolean modeIsSetPosition = false; // Otherwise set velocity

    public static double setPoint_S;
     public static double setPoint_W;

    public static boolean usingSmartDashboard = true;

    public static void setModePosition() {
        modeIsSetPosition = true;
    }

    public static void setModeVelocity() {
        modeIsSetPosition = false;
    }

    public void setShoulderPosition(double distance) {
        modeIsSetPosition = true;
        setPoint_S = distance;
      }

    public void setWristPosition(double distance) {
        modeIsSetPosition = true;
        setPoint_W = distance;
    }

    public static void PIDArmInit() {
        int smartMotionSlot = 0;

        CANSparkMax[] motorArray = { driveShoulder_R, driveShoulder_L, driveWrist };
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
            SmartDashboard.putNumber("P Gain", kP);
            SmartDashboard.putNumber("I Gain", kI);
            SmartDashboard.putNumber("D Gain", kD);
            SmartDashboard.putNumber("I Zone", kIz);
            SmartDashboard.putNumber("Feed Forward", kFF);
            SmartDashboard.putNumber("Max Output", kMaxOutput);
            SmartDashboard.putNumber("Min Output", kMinOutput);

            // display Smart Motion coefficients
            SmartDashboard.putNumber("Max Velocity", maxVel);
            SmartDashboard.putNumber("Min Velocity", minVel);
            SmartDashboard.putNumber("Max Acceleration", maxAcc);
            SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
            SmartDashboard.putNumber("Set Position", 0);
            SmartDashboard.putNumber("Set Velocity", 0);

            // button to toggle between velocity and smart motion modes
            SmartDashboard.putBoolean("Mode", true);
        }
    }

    public static void PIDArmUpdate() {
        if ((Math.abs(Robot.xboxController.getRightY()) > deadband)
                || (Math.abs(Robot.xboxController.getLeftY()) > deadband)) {
            setModeVelocity();
        }
        if (modeIsSetPosition) {
            driveShoulder_R.getPIDController().setReference(setPoint_S, CANSparkMax.ControlType.kSmartMotion);
            driveShoulder_L.getPIDController().setReference(-1 * setPoint_S, CANSparkMax.ControlType.kSmartMotion);  // -1 because the left motor turns the opposite way when the arm moves
            driveWrist.getPIDController().setReference(setPoint_W, CANSparkMax.ControlType.kSmartMotion);
        } else {
            if (Math.abs(Robot.xboxController.getLeftY()) > deadband) {
                setPoint_S = multiplier * Robot.xboxController.getLeftY();
                SmartDashboard.putNumber("Shoulder Setpoint", setPoint_S);
                driveShoulder_R.getPIDController().setReference(setPoint_S, CANSparkMax.ControlType.kVelocity);
                driveShoulder_L.getPIDController().setReference(-1 * setPoint_S, CANSparkMax.ControlType.kVelocity);
            }
            if (Math.abs(Robot.xboxController.getRightY()) > deadband) {
                setPoint_W = Robot.xboxController.getRightY();
                driveWrist.getPIDController().setReference(setPoint_W, CANSparkMax.ControlType.kVelocity);
            }
        }

        Update(driveShoulder_R);
        Update(driveShoulder_L);
        Update(driveWrist);
    }

    public static void Update(CANSparkMax motor) {
        SparkMaxPIDController pidController = motor.getPIDController();

        if (usingSmartDashboard) {
            double p = SmartDashboard.getNumber("P Gain", 0);
            double i = SmartDashboard.getNumber("I Gain", 0);
            double d = SmartDashboard.getNumber("D Gain", 0);
            double iz = SmartDashboard.getNumber("I Zone", 0);
            double ff = SmartDashboard.getNumber("Feed Forward", 0);
            double max = SmartDashboard.getNumber("Max Output", 0);
            double min = SmartDashboard.getNumber("Min Output", 0);
            double maxV = SmartDashboard.getNumber("Max Velocity", 0);
            double minV = SmartDashboard.getNumber("Min Velocity", 0);
            double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
            double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

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

    public static void updateTelemetry() {
        SmartDashboard.putNumber("Shoulder Right Encoder", driveShoulder_R.getEncoder().getPosition());
        SmartDashboard.putNumber("Shoulder Left Encoder", driveShoulder_L.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Encoder", driveWrist.getEncoder().getPosition());

        if (modeIsSetPosition) {
            SmartDashboard.putNumber("SetPoint_SR_Pos", setPoint_S);
            SmartDashboard.putNumber("SetPoint_SL_Pos", setPoint_S);
            SmartDashboard.putNumber("SetPoint_W_Pos", setPoint_W);

            SmartDashboard.putNumber("Encoder_SR_Pos", driveShoulder_R.getEncoder().getPosition());
            SmartDashboard.putNumber("Encoder_SL_Pos", driveShoulder_L.getEncoder().getPosition());
            SmartDashboard.putNumber("Encoder_W_Pos", driveWrist.getEncoder().getPosition());

            SmartDashboard.putNumber("MotorOutput_SR_Pos", driveShoulder_R.getAppliedOutput());
            SmartDashboard.putNumber("MotorOutput_SL_Pos", driveShoulder_L.getAppliedOutput());
            SmartDashboard.putNumber("MotorOutput_W_Pos", driveWrist.getAppliedOutput());
        } else {
            SmartDashboard.putNumber("SetPoint_SR_Vel", setPoint_S);
            SmartDashboard.putNumber("SetPoint_SL_Vel", setPoint_S);
            SmartDashboard.putNumber("SetPoint_W_Vel", setPoint_W);

            SmartDashboard.putNumber("Encoder_SR_Vel", driveShoulder_R.getEncoder().getVelocity());
            SmartDashboard.putNumber("Encoder_SL_Vel", driveShoulder_L.getEncoder().getVelocity());
            SmartDashboard.putNumber("Encoder_W_Vel", driveWrist.getEncoder().getVelocity());

            SmartDashboard.putNumber("MotorOutput_SR_Vel", driveShoulder_R.getAppliedOutput());
            SmartDashboard.putNumber("MotorOutput_SL_Vel", driveShoulder_L.getAppliedOutput());
            SmartDashboard.putNumber("MotorOutput_W_Vel", driveWrist.getAppliedOutput());
        }
    }
}

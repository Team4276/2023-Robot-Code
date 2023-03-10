package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public static boolean holdPosition;

    public static double setPoint;

    public static boolean usingSmartDashboard = false;

    public static void PIDDrivetrainInit() {
        int smartMotionSlot = 0;

        CANSparkMax[] motorArray = { frDriveX, flDriveX, brDriveX, blDriveX };
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

    public static void PIDDrivetrainUpdate() {
        SmartDashboard.putBoolean("holdPosition", holdPosition);

        Update(1, frDriveX);
        Update(-1, flDriveX);
        Update(1, brDriveX);
        Update(-1, blDriveX);
    }

    public static void Update(double sign, CANSparkMax motor) {
        SparkMaxPIDController pidController = motor.getPIDController();
        RelativeEncoder encoder = motor.getEncoder();
        
        if (usingSmartDashboard) {
            updatePID(pidController);
        }

        if (holdPosition) {
            if (newPositiontohold) {
                holdThisPosition = encoder.getPosition();
                newPositiontohold = false;
            }

            pidController.setReference(sign*holdThisPosition, CANSparkMax.ControlType.kSmartMotion);

            if (usingSmartDashboard) {
                SmartDashboard.putNumber("holdThisPosition", holdThisPosition);
                SmartDashboard.putNumber("SetPoint", holdThisPosition);
                SmartDashboard.putNumber("Process Variable", encoder.getPosition());
            }

        } else {
            double processVariable;
            newPositiontohold = true;
            pidController.setReference(setPoint * sign, CANSparkMax.ControlType.kVelocity);
            processVariable = encoder.getVelocity();
            
            if (usingSmartDashboard) {
                SmartDashboard.putNumber("SetPoint", setPoint);
                SmartDashboard.putNumber("Process Variable", processVariable);
            }

        }
        if (usingSmartDashboard) {
            SmartDashboard.putNumber("Output", motor.getAppliedOutput());
        }
    }

    public static void updatePID(SparkMaxPIDController pidController){
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

    public static void updateTelemetry() {
        updateBaseTelemetry();
    }

}
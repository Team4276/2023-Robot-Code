package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.Xbox;

public class PIDElbow extends Elbow {

    public PIDElbow(int port) {
        super(port);
        //TODO Auto-generated constructor stub
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
    private static double deadband = 0.2;

    private static boolean usingSmartDashboard = false;
    public static boolean modeIsSetPosition = false; // Otherwise set velocity

    public static double setPoint_W;

    public static void setModePosition() {
        modeIsSetPosition = true;
    }

    public static void setModeVelocity() {
        modeIsSetPosition = false;
    }

    private static void setPIDReference(double setPoint_W){
        SmartDashboard.putNumber("Set Position", setPoint_W);
        driveElbow.getPIDController().setReference(setPoint_W, CANSparkMax.ControlType.kSmartMotion);
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

        // display POV coefficient 
        SmartDashboard.putNumber("POV", Robot.xboxController.getPOV());
    }

    public static void PIDElbowUpdate() {
        if ((Math.abs(Robot.xboxController.getRightY()) > deadband)) {
            setModeVelocity();
        } else {
            setModePosition();
        }

        if (modeIsSetPosition) {
            if (Robot.pov != -1){

                if (Xbox.POVup == Robot.pov){
                    setPoint_W = 7;
                    setPIDReference(setPoint_W);
                } else if (Xbox.POVdown == Robot.pov){
                    setPoint_W = -7;
                    setPIDReference(setPoint_W);
                } else if (Xbox.POVright == Robot.pov){
                    setPoint_W = 2;
                    setPIDReference(setPoint_W);
                } else if (Xbox.POVleft == Robot.pov){
                    setPoint_W = -2;
                    setPIDReference(setPoint_W);
                }
            }
        }

        if (usingSmartDashboard){
            Update(driveElbow);
        }
    }

    public static void Update(CANSparkMax motor) {
        SparkMaxPIDController pidController = motor.getPIDController();
        
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

    public static void updateTelemetry(){
        SmartDashboard.putNumber("SetPoint_W_Pos", setPoint_W);

        SmartDashboard.putNumber("Encoder_W_Pos", driveElbow.getEncoder().getPosition());

        SmartDashboard.putNumber("MotorOutput_W_Pos", driveElbow.getAppliedOutput());
    }
}

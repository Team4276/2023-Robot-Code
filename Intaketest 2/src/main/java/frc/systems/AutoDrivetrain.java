package frc.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.reflect.Array;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class AutoDrivetrain {
    public static boolean holdPosition;
    
    private static SparkMaxPIDController FR_pidController, FL_pidController, BR_pidController, BL_pidController;
    public static RelativeEncoder FR_encoder, FL_encoder, BR_encoder, BL_encoder;

    private static CANSparkMax FR_motor = BaseDrivetrain.frDriveX;
    private static CANSparkMax FL_motor = BaseDrivetrain.flDriveX;
    private static CANSparkMax BR_motor = BaseDrivetrain.brDriveX;
    private static CANSparkMax BL_motor = BaseDrivetrain.blDriveX;

    // PID coefficients
    public static double kP = 5e-5; 
    public static double kI = 1e-6;
    public static double kD = 0; 
    public static double kIz = 0; 
    public static double kFF = 0.000156; 
    public static double kMaxOutput = 1; 
    public static double kMinOutput = -1;
    public static double maxRPM = 1000;

    // Smart Motion Coefficients
    public static double maxVel = 1000; // rpm
    public static double maxAcc = 100;
    public static double minVel = 0;

    public static double allowedErr = 0;

    public static Array motorArray;

    public static void PIDDrivetrainInit(){
        int smartMotionSlot = 0;

        CANSparkMax[] motorArray = {FR_motor, FL_motor, BR_motor, BL_motor};
        for (CANSparkMax motor : motorArray){
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


        FR_pidController = FR_motor.getPIDController();
        FL_pidController = FL_motor.getPIDController();
        BR_pidController = BR_motor.getPIDController();
        BL_pidController = BL_motor.getPIDController();

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

    public static void PIDDrivetrainUpdate(){
        SmartDashboard.putBoolean("usingAutoDrivetrain", BaseDrivetrain.usingAutoDrivetrain);
        SmartDashboard.putBoolean("holdPosition", holdPosition);

        if (BaseDrivetrain.usingAutoDrivetrain) {
            Update(FR_pidController, BaseDrivetrain.FR_encoder, 1, BaseDrivetrain.frDriveX, holdPosition);
            Update(FL_pidController, BaseDrivetrain.FL_encoder, -1, BaseDrivetrain.flDriveX, holdPosition);
            Update(BR_pidController, BaseDrivetrain.BR_encoder, 1, BaseDrivetrain.brDriveX, holdPosition);
            Update(BL_pidController, BaseDrivetrain.BL_encoder, -1, BaseDrivetrain.blDriveX, holdPosition);
        }
    }

    public static void Update(SparkMaxPIDController m_pidController, RelativeEncoder m_encoder, double sign, CANSparkMax m_motor, boolean holdPosition){
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

        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            m_pidController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }
        if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
        if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
        if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
        if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

        if (holdPosition){
            double currentPosition = m_encoder.getPosition();

            m_pidController.setReference(currentPosition, CANSparkMax.ControlType.kSmartMotion);
            SmartDashboard.putNumber("SetPoint", currentPosition);
            SmartDashboard.putNumber("Process Variable", m_encoder.getPosition());

        } else {
            double setPoint, processVariable;
            boolean mode = SmartDashboard.getBoolean("Mode", false);
            if(mode) {
                setPoint = sign * SmartDashboard.getNumber("Set Velocity", 0);
                m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
                processVariable = m_encoder.getVelocity();
            } else {
                setPoint = sign * SmartDashboard.getNumber("Set Position", 0);
                /**
                 * As with other PID modes, Smart Motion is set by calling the
                 * setReference method on an existing pid object and setting
                 * the control type to kSmartMotion
                 */
                m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
                processVariable = m_encoder.getPosition();
                
            }
            
            SmartDashboard.putNumber("SetPoint", setPoint);
            SmartDashboard.putNumber("Process Variable", processVariable);

        }
        SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());
    }

}
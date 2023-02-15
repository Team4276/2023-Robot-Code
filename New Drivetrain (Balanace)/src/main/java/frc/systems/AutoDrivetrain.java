package frc.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class AutoDrivetrain extends BaseDrivetrain {
    public AutoDrivetrain(int FLport, int BLport, int FRport, int BRport, int m_right_encoderPortA,
    int m_right_encoderPortB, int m_left_encoderPortA, int m_left_encoderPortB) {
        
    super(FLport, BLport, FRport, BRport, m_right_encoderPortA, m_right_encoderPortB, m_left_encoderPortA,
    m_left_encoderPortB);
// TODO Auto-generated constructor stub
}
    private static SparkMaxPIDController FR_pidController, FL_pidController, BR_pidController, BL_pidController;

    private static CANSparkMax FR_motor = BaseDrivetrain.frDriveX;
    private static CANSparkMax FL_motor = BaseDrivetrain.flDriveX;
    private static CANSparkMax BR_motor = BaseDrivetrain.brDriveX;
    private static CANSparkMax BL_motor = BaseDrivetrain.blDriveX;

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
    
    private static boolean newPositiontohold = true;

    public static boolean holdPosition;
    public static boolean mode;

    public static double setPoint;

    public static boolean usingSmartDashboard = false;

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
            Update(FR_pidController, BaseDrivetrain.FR_encoder, 1, FR_motor);
            Update(FL_pidController, BaseDrivetrain.FL_encoder, -1, FL_motor);
            Update(BR_pidController, BaseDrivetrain.BR_encoder, 1, BR_motor);
            Update(BL_pidController, BaseDrivetrain.BL_encoder, -1, BL_motor);
        }
    }

    public static void Update(SparkMaxPIDController m_pidController, RelativeEncoder m_encoder, double sign, CANSparkMax m_motor){
        if (usingSmartDashboard){
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
        }

        if (holdPosition){
            if (newPositiontohold)
                holdThisPosition = m_encoder.getPosition();
                SmartDashboard.putNumber("holdThisPosition", holdThisPosition);
                newPositiontohold = false;

            m_pidController.setReference(holdThisPosition, CANSparkMax.ControlType.kSmartMotion);
            SmartDashboard.putNumber("SetPoint", holdThisPosition);
            SmartDashboard.putNumber("Process Variable", m_encoder.getPosition());

        } else {
            double processVariable;
            newPositiontohold = true;
            if(mode) {
                m_pidController.setReference(setPoint * sign, CANSparkMax.ControlType.kVelocity);
                processVariable = m_encoder.getVelocity();
            } else {
                /**
                 * As with other PID modes, Smart Motion is set by calling the
                 * setReference method on an existing pid object and setting
                 * the control type to kSmartMotion
                 */
                m_pidController.setReference(setPoint * sign, CANSparkMax.ControlType.kSmartMotion);
                processVariable = m_encoder.getPosition();
                
            }
            
            SmartDashboard.putNumber("SetPoint", setPoint);
            SmartDashboard.putNumber("Process Variable", processVariable);

        }
        SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());
    }

}
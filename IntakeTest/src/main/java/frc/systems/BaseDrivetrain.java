package frc.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class BaseDrivetrain {
     public static RelativeEncoder FR_encoder, FL_encoder, BR_encoder, BL_encoder;
    
    public static CANSparkMax flDriveX, blDriveX, frDriveX, brDriveX;

    public static Encoder m_left_encoder;
    public static Encoder m_right_encoder;


    public BaseDrivetrain(int FLport, int BLport, int FRport, int BRport, int m_right_encoderPortA, 
    int m_right_encoderPortB, int m_left_encoderPortA, int m_left_encoderPortB){
            flDriveX = new CANSparkMax(FLport, MotorType.kBrushless);
            blDriveX = new CANSparkMax(BLport, MotorType.kBrushless);
            frDriveX = new CANSparkMax(FRport, MotorType.kBrushless);
            brDriveX = new CANSparkMax(BRport, MotorType.kBrushless);
            
            FR_encoder = frDriveX.getEncoder();
            FL_encoder = flDriveX.getEncoder();
            BR_encoder = brDriveX.getEncoder();
            BL_encoder = blDriveX.getEncoder();

    
    }

    public static void updateTelemetry(){
        SmartDashboard.putNumber("Front Right Encoder", FR_encoder.getPosition());
        SmartDashboard.putNumber("Front Left Encoder", FL_encoder.getPosition());
        SmartDashboard.putNumber("Back Right Encoder", BR_encoder.getPosition());
        SmartDashboard.putNumber("Back Left Encoder", BL_encoder.getPosition());

    }
}

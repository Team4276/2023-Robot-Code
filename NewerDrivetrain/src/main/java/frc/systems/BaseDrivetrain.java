package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BaseDrivetrain {
    public static CANSparkMax flDriveX = null;
    public static CANSparkMax blDriveX = null;
    public static CANSparkMax frDriveX = null;
    public static CANSparkMax brDriveX = null;

    public BaseDrivetrain(int FLport, int BLport, int FRport, int BRport) {
        if (flDriveX==null){
            flDriveX = new CANSparkMax(FLport, MotorType.kBrushless);
        }
        if (blDriveX==null){
            blDriveX = new CANSparkMax(BLport, MotorType.kBrushless);
        }
        if (frDriveX==null){
            frDriveX = new CANSparkMax(FRport, MotorType.kBrushless);
        }
        if (brDriveX==null){
            brDriveX = new CANSparkMax(BRport, MotorType.kBrushless);
        }
    }

    public static void updateBaseTelemetry() {
        SmartDashboard.putNumber("Front Right Encoder", frDriveX.getEncoder().getPosition());
        SmartDashboard.putNumber("Front Left Encoder", flDriveX.getEncoder().getPosition());
        SmartDashboard.putNumber("Back Right Encoder", brDriveX.getEncoder().getPosition());
        SmartDashboard.putNumber("Back Left Encoder", blDriveX.getEncoder().getPosition());
    }
}

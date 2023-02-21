package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BaseDrivetrain {
    public static CANSparkMax flDriveX, blDriveX, frDriveX, brDriveX;

    public BaseDrivetrain(int FLport, int BLport, int FRport, int BRport) {
        flDriveX = new CANSparkMax(FLport, MotorType.kBrushless);
        blDriveX = new CANSparkMax(BLport, MotorType.kBrushless);
        frDriveX = new CANSparkMax(FRport, MotorType.kBrushless);
        brDriveX = new CANSparkMax(BRport, MotorType.kBrushless);
    }

    public static void updateBaseTelemetry() {
        SmartDashboard.putNumber("Front Right Encoder", frDriveX.getEncoder().getPosition());
        SmartDashboard.putNumber("Front Left Encoder", flDriveX.getEncoder().getPosition());
        SmartDashboard.putNumber("Back Right Encoder", brDriveX.getEncoder().getPosition());
        SmartDashboard.putNumber("Back Left Encoder", blDriveX.getEncoder().getPosition());
    }
}

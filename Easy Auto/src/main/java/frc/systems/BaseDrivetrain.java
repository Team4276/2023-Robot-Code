package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

        flDriveX.getEncoder().setPosition(0.0);
        frDriveX.getEncoder().setPosition(0.0);
        blDriveX.getEncoder().setPosition(0.0);
        brDriveX.getEncoder().setPosition(0.0);
    }

}

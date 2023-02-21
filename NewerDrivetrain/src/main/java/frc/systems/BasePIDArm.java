package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BasePIDArm {
    public static RelativeEncoder SR_encoder, SL_encoder, W_encoder;

    private static CANSparkMax driveShoulder_R;
    private static CANSparkMax driveShoulder_L;
    private static CANSparkMax driveWrist;

    public BasePIDArm(int SRport, int SLport, int Wportt) {
        driveShoulder_R = new CANSparkMax(SRport, MotorType.kBrushless);
        driveShoulder_L = new CANSparkMax(SLport, MotorType.kBrushless);
        driveWrist = new CANSparkMax(Wportt, MotorType.kBrushless);

        SR_encoder = driveShoulder_R.getEncoder();
        SL_encoder = driveShoulder_L.getEncoder();
        W_encoder = driveWrist.getEncoder();
    }

    public static void updateBaseTelemetry() {
        SmartDashboard.putNumber("Front Right Encoder", SR_encoder.getPosition());
        SmartDashboard.putNumber("Shoulder Left Encoder", SL_encoder.getPosition());
        SmartDashboard.putNumber("Shoulder Right Encoder", W_encoder.getPosition());
    }
}

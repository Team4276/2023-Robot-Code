package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.Xbox;

public class GodplsworkShoulder {
    public static CANSparkMax driveShoulder_R;
    public static CANSparkMax driveShoulder_L;

    public static double deadband = 0.2;

    public GodplsworkShoulder(int port_R, int port_L) {
        driveShoulder_R.restoreFactoryDefaults();
        driveShoulder_L.restoreFactoryDefaults();

        driveShoulder_R = new CANSparkMax(port_R, MotorType.kBrushless);
        driveShoulder_L = new CANSparkMax(port_L, MotorType.kBrushless);

        driveShoulder_L.follow(driveShoulder_R);
        driveShoulder_L.setInverted(true);

    }

    public static void GodplsworkShoulderUpdate(){
        
        if(Robot.xboxController.getRightY() < deadband){
            driveShoulder_R.set(0);

        } else if(Robot.xboxController.getRightY() > deadband){
            double power = Robot.xboxController.getRightY()/100;

            driveShoulder_R.set(power);
        }

        SmartDashboard.putNumber("ShoulderEncoder_R:  ", driveShoulder_R.getEncoder().getPosition());
    }

    
}

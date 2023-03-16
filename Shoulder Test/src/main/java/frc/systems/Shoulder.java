package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Shoulder {
    public static CANSparkMax driveShoulder_R;
    public static CANSparkMax driveShoulder_L;

    public static double deadband = 0.2;

    public Shoulder(int port_R, int port_L) {
        driveShoulder_R = new CANSparkMax(port_R, MotorType.kBrushless);
        driveShoulder_L = new CANSparkMax(port_L, MotorType.kBrushless);

        driveShoulder_R.restoreFactoryDefaults();
        driveShoulder_L.restoreFactoryDefaults();

        driveShoulder_L.follow(driveShoulder_R, true);

        driveShoulder_R.burnFlash();
        driveShoulder_L.burnFlash();

    }

    public static void ShoulderUpdate(){
        
        if(Math.abs(Robot.xboxController.getRightY()) < deadband){
            driveShoulder_R.set(0);

        } else if(Math.abs(Robot.xboxController.getRightY()) > deadband){
            double power = Robot.xboxController.getRightY()/3;

            driveShoulder_R.set(power);
        }

        SmartDashboard.putNumber("R Shoulder Applied Output", driveShoulder_R.getAppliedOutput());
        SmartDashboard.putNumber("L Shoulder Applied Output", driveShoulder_L.getAppliedOutput());
    }

    
}

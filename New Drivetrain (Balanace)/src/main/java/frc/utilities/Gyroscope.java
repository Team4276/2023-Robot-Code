package frc.utilities;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

public class Gyroscope {
    public static final ADIS16470_IMU imu = new ADIS16470_IMU();

    static public double GetYaw() {

        return imu.getAngle();
    }

    static public double GetRoll() {

        return imu.getXComplementaryAngle();
    }

    
    static public double GetPitch() {
        return imu.getYComplementaryAngle();
    }

    static public double GetCorrectPitch(double currentPitch) {
        double Pitch = imu.getYComplementaryAngle();

        Pitch = currentPitch - Robot.initialPitch;

        return Pitch;
    }

    static public void gyroscopeUpdate(){
        SmartDashboard.putNumber("pitch", Gyroscope.GetPitch());
        SmartDashboard.putNumber("corrected pitch", Gyroscope.GetCorrectPitch(Gyroscope.GetPitch()));
    }
}

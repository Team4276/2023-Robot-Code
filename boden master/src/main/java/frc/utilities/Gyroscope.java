package frc.utilities;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

import frc.robot.Robot;

public class Gyroscope {
    public static final ADIS16470_IMU imu = new ADIS16470_IMU();

    static public double GetYaw() {
        double yaw = imu.getAngle() % 360;
        if (yaw < 0){
        yaw = imu.getAngle() % 360 * -1;
        }
        return yaw;
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

}

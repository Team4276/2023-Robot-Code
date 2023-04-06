package frc.utilities;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class Gyroscope {
    public static final ADIS16470_IMU imu = new ADIS16470_IMU();

    public static double initialPitch = 0;
    public static double initialYaw = 0;

    public static void gyroscopeInit(){
        initialPitch = GetPitch();
        initialYaw = GetYaw();
    }

    static public double GetYaw() {

        return imu.getAngle();
    }

    static public double GetRoll() {

        return imu.getXComplementaryAngle();
    }

    static public double GetPitch() {
        return imu.getYComplementaryAngle();
    }

    static public double GetCorrectPitch() {
        double Pitch = imu.getYComplementaryAngle();

        Pitch -= initialPitch;

        return Pitch;
    }

    static public double GetCorrectedYaw(){
        double Yaw = imu.getAngle() % 360;

        Yaw -= initialYaw;

        return Yaw;

    }

}

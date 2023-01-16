package frc.utilities;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class Gyroscope {
    public static final ADIS16470_IMU imu = new ADIS16470_IMU();

    static public double ErrorCorrection() {
        //placeholder for error buildup correction code

        return 0;

    }

    static public double GetYaw() {

        return imu.getAngle();
    }

    static public double GetPitch() {

        return imu.getXComplementaryAngle();
    }

    static public double GetRoll() {

        return imu.getYComplementaryAngle();
    }


}

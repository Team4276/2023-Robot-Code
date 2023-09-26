package frc.utils;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class Gyroscope {
    ADIS16470_IMU gyro;

    public Gyroscope(ADIS16470_IMU gyro){
        this.gyro = gyro;

    }

    public double GetYaw() {
        return gyro.getAngle();
    }

    public double GetRoll() {
        return gyro.getXComplementaryAngle();
    }

    public double GetPitch() {
        return gyro.getYComplementaryAngle();
        
    }
}


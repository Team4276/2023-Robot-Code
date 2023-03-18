package frc.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.systems.BaseDrivetrain;

public class Location4276 {

    private static NetworkTable ntLimelight;

    // Limelight
    private static Vector3 v3Limelight;
    private static Vector3 v3PreviousLimelight;

    private static Vector3 v3Position;
    private static Vector3 v3PrevPosition;
    private static int nUpdateCount = 0;

    private double gyroCorrection = 0.0;
    private long positionUpdateTimeNanosecs;

    public Location4276() {

        v3Limelight = new Vector3(0.0, 0.0, 0.0);
        v3PreviousLimelight = new Vector3(0.0, 0.0, 0.0);
        v3Position = new Vector3(0.0, 0.0, 0.0);
        v3PrevPosition = new Vector3(0.0, 0.0, 0.0);
    
        ntLimelight = NetworkTableInstance.getDefault().getTable("limelight");

        double[] errorhandle = new double[6];
        double[] positionLimelight = ntLimelight.getEntry("botpose").getDoubleArray(errorhandle);
        double x = positionLimelight[0];
        double y = positionLimelight[1];
        double  z= positionLimelight[2];
        v3Limelight.set(x,y,z);

        positionUpdateTimeNanosecs = java.lang.System.nanoTime();
    }

    public boolean isNewPositionFix() {
        return (!v3Limelight.isEqual(v3PreviousLimelight));
    }

    public void setPositionFix() {
        v3PreviousLimelight.copy(v3Limelight);

        v3Position.copy(v3Limelight);
    }

    private double getHeading() {
        return Gyroscope.GetYaw() - gyroCorrection;
    }

    public double getEncoderSpeed() {
        // Find the lowest speed encoder (assume faster speed indicates slippage or
        // minor difference due to turning)

        double speed = 0.0;
        double BRencoder = BaseDrivetrain.brDriveX.getEncoder().getVelocity();
        double BLencoder = BaseDrivetrain.blDriveX.getEncoder().getVelocity();
        double FRencoder = BaseDrivetrain.flDriveX.getEncoder().getVelocity();
        double FLencoder = BaseDrivetrain.frDriveX.getEncoder().getVelocity();

        // 2023 robot can sense velocity directly from the SparkMAX controller
        if (speed > BRencoder) {
            speed = BRencoder;
        }
        if (speed > (-1 * BLencoder)) {
            speed = (-1 * BLencoder);
        }
        if (speed > FRencoder) {
            speed = FRencoder;
        }
        if (speed > (-1 * FLencoder)) {
            speed = (-1 * FLencoder);
        }

        return speed;
    }

    public void updatePosition() {

        if (isNewPositionFix()) {
            setPositionFix();
        } else {
            // Limelight did not find any Apriltag

            nUpdateCount++;
            if (nUpdateCount > 10) {
                nUpdateCount = 0;

                // heading of robot moving from previous position to current position
                // Y axis == Robot Forward, X == Robot right
                // 0.0 heading == Robot forward, Positive rotation to Robot Right, range -180.0
                // to +180.0
                double estimateCourseMadeGood = v3PrevPosition.angle(v3Position);
                gyroCorrection = Gyroscope.GetYaw() - estimateCourseMadeGood;
            }
 
            // Estimate current heading and speed to extrapolate current position from
            // previous position

            long prevTicks = positionUpdateTimeNanosecs;
            positionUpdateTimeNanosecs = java.lang.System.nanoTime();
            long deltaTimeNanosecs = (positionUpdateTimeNanosecs - prevTicks);

            double heading = getHeading();
            double speed = getEncoderSpeed();
            double distance = speed * deltaTimeNanosecs;
          
            v3PrevPosition.copy(v3Limelight);

            v3Position.x = distance * Math.cos(heading);
            v3Position.y = distance * Math.sin(heading);
        }
    }

    public void updateTelemetry() {
        double heading = getHeading();
        SmartDashboard.putNumberArray("CurrentPos_XYZ: ", v3Position.getDoubleArray());
        SmartDashboard.putNumber("Heading: ", heading);
        SmartDashboard.putNumber("Speed: ", getEncoderSpeed());
    }
}

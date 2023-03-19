package frc.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.systems.BaseDrivetrain;

public class Location4276 {

    private static NetworkTable ntLimelight;

    // Limelight
    private static Vector3 v3Limelight;
    private static Vector3 v3PreviousLimelight;

    private static Vector3 v3Position;
    private static Vector3 v3PrevPosition;
    private static int nUpdateCount = 0;

    private static double heading = 0.0;
    private static double speed = 0.0;
    private static double distance = 0.0;

    private double gyroCorrection = 0.0;
    private long positionUpdateTimeMillisecs;

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
        double z = positionLimelight[2];
        v3Limelight.set(x, y, z);

        positionUpdateTimeMillisecs = java.lang.System.currentTimeMillis();
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

    private double getDistanceTo(Vector3 otherPos) {
        return Math.sqrt(Math.pow((v3Position.x - otherPos.x), 2)
                + Math.pow((v3Position.y - otherPos.y), 2));
    }

    public double getEncoderSpeed() {
        // Find the lowest speed encoder (assume faster speed indicates slippage or
        // minor difference due to turning)

        speed = 0.0;

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
            Robot.myLogFile.write("X,");
            distance = getDistanceTo(v3Limelight);
            setPositionFix();

        } else {
            // Limelight did not find any Apriltag
            Robot.myLogFile.write("p,");

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

            long prevTimeMillisecs = positionUpdateTimeMillisecs;
            positionUpdateTimeMillisecs = java.lang.System.nanoTime();
            long deltaTimeMillisecs = (positionUpdateTimeMillisecs - prevTimeMillisecs);

            heading = getHeading();
            speed = getEncoderSpeed();
            distance = speed * (1000.0 * deltaTimeMillisecs);

            v3PrevPosition.copy(v3Position);

            v3Position.x = distance * Math.sin(heading);
            v3Position.y = distance * Math.cos(heading);
        }

        Robot.myLogFile.write(String.valueOf(distance));
        Robot.myLogFile.write(String.valueOf(","));
        Robot.myLogFile.write(String.valueOf(speed));
        Robot.myLogFile.write(String.valueOf(","));
        Robot.myLogFile.write(String.valueOf(heading));
        Robot.myLogFile.write(String.valueOf(","));
        Robot.myLogFile.write(String.valueOf(v3Position.x));
        Robot.myLogFile.write(String.valueOf(","));
        Robot.myLogFile.write(String.valueOf(v3Position.y));
        Robot.myLogFile.write(String.valueOf(","));
        Robot.myLogFile.write(String.valueOf(v3Position.z));
        Robot.myLogFile.write(String.valueOf("\r\n"));

    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("CurrentPos_X: ", v3Position.x);
        SmartDashboard.putNumber("CurrentPos_Y: ", v3Position.y);
        SmartDashboard.putNumber("Heading: ", heading);
        SmartDashboard.putNumber("Speed: ", speed);
    }
}

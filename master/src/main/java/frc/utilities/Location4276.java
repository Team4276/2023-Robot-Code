package frc.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.systems.BaseDrivetrain;

public class Location4276 {

    // Limelight
    private static Vector3 v3Limelight;
    private static Vector3 v3PreviousLimelight;

    private static Vector3 v3Position;
    private static Vector3 v3PrevPosition;

    private static double heading = 0.0;
    private static double speed = 0.0;
    private static double distance = 0.0;
    private static double posFixErrorCorrection = 0.0;

    private static double gyroCorrection = 0.0;
    private static long positionUpdateTimeMillisecs;

    public Location4276() {

        v3Limelight = new Vector3(0.0, 0.0, 0.0);
        v3PreviousLimelight = new Vector3(0.0, 0.0, 0.0);
        v3Position = new Vector3(0.0, 0.0, 0.0);
        v3PrevPosition = new Vector3(0.0, 0.0, 0.0);

        checkLimelightRobotPosition();
    }

    public void checkLimelightRobotPosition() {

        final double feet_per_meter = 3.28084;

        double[] errorhandle = new double[6];
        double[] positionLimelight = Robot.ntLimelight.getEntry("botpos").getDoubleArray(errorhandle);
        double x = positionLimelight[0];
        double y = positionLimelight[1];
        double z = positionLimelight[2];

        // position units are meters at this point - convert to feet
        x *= feet_per_meter;
        y *= feet_per_meter;
        z *= feet_per_meter;

        v3Limelight.set(x, y, z);
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

        // Robot coordinates reported by Limelight
        // 3d Cartesian Coordinate System with (0,0,0) located at the center of the
        // robot's frame projected down to the floor.
        // X Pointing forward (Forward Vector)
        // Y Pointing toward the robot's right (Right Vector)
        // Z Pointing upward (Up Vector)

        // This is different from the coordinate system used by the Gyro (X and Y axis
        // swapped)
        // X Pointing toward the robot's right (Right Vector)
        // Y Pointing forward (Forward Vector)
        // Z Pointing upward (Up Vector)

        // I am guessing that it doesn't matter for heading because rotation is around
        // the Z-axis which is same for both
    }

    private double getDistanceTo(Vector3 otherPos) {
        return Math.sqrt(Math.pow((v3Position.x - otherPos.x), 2)
                + Math.pow((v3Position.y - otherPos.y), 2));
    }

    public double getEncoderSpeed() {
        // Find the lowest speed encoder (assume faster speed indicates slippage or
        // minor difference due to turning)

        double rpmSpeed = 10000000.0;

        double BRencoder = BaseDrivetrain.brDriveX.getEncoder().getVelocity();
        double BLencoder = BaseDrivetrain.blDriveX.getEncoder().getVelocity();
        double FRencoder = BaseDrivetrain.flDriveX.getEncoder().getVelocity();
        double FLencoder = BaseDrivetrain.frDriveX.getEncoder().getVelocity();

        // 2023 robot can sense velocity directly from the SparkMAX controller
        if (rpmSpeed > FRencoder) {  
            rpmSpeed = FRencoder;
        }
        if (rpmSpeed > (-1 * BRencoder)) { 
            rpmSpeed = (-1 * BRencoder);
        }
        if (rpmSpeed > (-1 * FLencoder)) { 
            rpmSpeed = (-1 * FLencoder);
        }
        if (rpmSpeed > BLencoder) {
            rpmSpeed = BLencoder;
        }

        // Speed units are rpm at this point - need to convert to feet/sec:
        // 8.5:1 gearboxes, on the new wheels that are a little over 3in radius
        // (100 rotations)/(8.5 gearbox redux) * (2pi*3.05in) = 225.5in, or 18.79ft/100
        // rotations
        final double convertRpmToFeet = 0.1879;
        rpmSpeed *= convertRpmToFeet;    // feet/min
        rpmSpeed /= 60.0;                // feet/sec

        return rpmSpeed;
    }

    private boolean isMotionSufficientToEstimateHeading() {
        return (getEncoderSpeed() > 1.0); // feet/sec
    }

    public void updatePosition() {

        checkLimelightRobotPosition();

        if (isNewPositionFix()) {
            posFixErrorCorrection = getDistanceTo(v3Limelight);
            setPositionFix();

        } else {
            // Limelight did not find any Apriltag
            posFixErrorCorrection = 0.0;

            if (isMotionSufficientToEstimateHeading()) {
                // heading of robot moving from previous position to current position
                // Y axis == Robot Forward, X == Robot right
                // 0.0 heading == Robot forward, Positive rotation to Robot Right, range -180.0
                // to +180.0
                double estimateCourseMadeGood = v3PrevPosition.angle(v3Position);
                gyroCorrection = estimateCourseMadeGood - Gyroscope.GetYaw();
            }

            // Extrapolate current position from previous position

            long prevTimeMillisecs = positionUpdateTimeMillisecs;
            positionUpdateTimeMillisecs = java.lang.System.currentTimeMillis();
            long deltaTimeMillisecs = (positionUpdateTimeMillisecs - prevTimeMillisecs);

            heading = getHeading();
            speed = getEncoderSpeed();
            distance = speed * (deltaTimeMillisecs/1000.0);

            SmartDashboard.putNumber("distance", distance);
            SmartDashboard.putNumber("DektaTimeMillis", deltaTimeMillisecs);

            v3PrevPosition.copy(v3Position);

            v3Position.x = distance * Math.sin(heading);
            v3Position.y = distance * Math.cos(heading);
        }

        if (getEncoderSpeed() != 0.0) { // No point in filling the log with duplicate data, timestanp will show periods
                                        // of stillness
            Robot.myLogFile.write(String.valueOf(posFixErrorCorrection));
            Robot.myLogFile.write(String.valueOf(","));
            Robot.myLogFile.write(String.valueOf(System.currentTimeMillis()));
            Robot.myLogFile.write(String.valueOf(","));
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
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("CurrentPos_X: ", v3Position.x);
        SmartDashboard.putNumber("CurrentPos_Y: ", v3Position.y);
        SmartDashboard.putNumber("Heading: ", heading);
        SmartDashboard.putNumber("Speed: ", speed);
    }
}

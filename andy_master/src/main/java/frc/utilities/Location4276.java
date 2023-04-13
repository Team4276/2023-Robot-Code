package frc.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.systems.BaseDrivetrain;

public class Location4276 {

    // PhotonVision
    private static Vector3 v3PhotonVision;
    private static Vector3 v3PreviousPhotonVision;

    private static Vector3 v3Position;
    private static Vector3 v3PrevPosition;

    private static double heading = 0.0;
    private static double speed = 0.0;
    private static double distance = 0.0;
    private static double posFixErrorCorrection = 0.0;

    private static double gyroCorrection = 0.0;
    private static double prevPosition = 0.0;

    public Location4276() {

        v3PhotonVision = new Vector3(0.0, 0.0, 0.0);
        v3PreviousPhotonVision = new Vector3(0.0, 0.0, 0.0);
        v3Position = new Vector3(0.0, 0.0, 0.0);
        v3PrevPosition = new Vector3(0.0, 0.0, 0.0);

        final double convertOneRevolutionToFeet = 0.1879;
        final double convertFeetToMeters = 0.3048;

        double kNeoPositionConversionFactor = convertOneRevolutionToFeet;
        kNeoPositionConversionFactor *= convertFeetToMeters;

        BaseDrivetrain.flDriveX.getEncoder().setPositionConversionFactor(kNeoPositionConversionFactor);
        BaseDrivetrain.frDriveX.getEncoder().setPositionConversionFactor(kNeoPositionConversionFactor);
    }

    public Pose2d getRobotPosition() {
        Rotation2d rot2d = Rotation2d.fromDegrees(heading);
        return new Pose2d(v3Position.x, v3Position.y, rot2d);
    }

    public boolean isNewPositionFix() {
        return (!v3PhotonVision.isEqual(v3PreviousPhotonVision));
    }

    public void setPositionFix() {
        v3PreviousPhotonVision.copy(v3PhotonVision);
        v3Position.copy(v3PhotonVision);
    }

    public static double getRawHeading() {
        double retVal = Gyroscope.GetYaw(); // starts at zero, negative is CCW from top,
        retVal %= 360; // now in range -360 to +360

        if (retVal > 180) {
            retVal -= 360;
        }
        if (retVal < -180) {
            retVal += 360;
        }
        return retVal;
    }

    private static double getHeading() {
        double retVal = getRawHeading();
        retVal += gyroCorrection;
        return retVal;
    }

    private double getDistanceTo(Vector3 otherPos) {
        return Math.sqrt(Math.pow((v3Position.x - otherPos.x), 2)
                + Math.pow((v3Position.y - otherPos.y), 2));
    }

    public double getEncoderSpeed() {
        // Find the lowest speed encoder (assume faster speed indicates slippage or
        // minor difference due to turning)

        double FRencoderSpeed = BaseDrivetrain.flDriveX.getEncoder().getVelocity();
        double FLencoderSpeed = BaseDrivetrain.frDriveX.getEncoder().getVelocity();

        double rpmSpeed = 10000000.0;
        if (rpmSpeed > FRencoderSpeed) {
            rpmSpeed = FRencoderSpeed;
        }
        if (rpmSpeed > (-1 * FLencoderSpeed)) {
            rpmSpeed = (-1 * FLencoderSpeed);
        }

        // Speed units are rpm at this point - need to convert to feet/sec:
        // 8.5:1 gearboxes, on the new wheels that are a little over 3in radius
        // (100 rotations)/(8.5 gearbox redux) * (2pi*3.05in) = 225.5in, or 18.79ft/100
        // rotations
        final double convertRpmToFeet = 0.1879;
        final double convertFeetToMeters = 0.3048;
        rpmSpeed *= convertRpmToFeet; // feet/min
        rpmSpeed *= convertFeetToMeters; // meters/min
        rpmSpeed /= 60.0; // meters/sec

        return rpmSpeed;
    }

    public double getEncoderDistance(double prevPos) {
        // Find the lowest speed encoder (assume faster speed indicates slippage or
        // minor difference due to turning)

        double FRencoderPosition = BaseDrivetrain.flDriveX.getEncoder().getPosition();
        double FLencoderPosition = BaseDrivetrain.frDriveX.getEncoder().getPosition();

        double pos = 10000000.0;
        if (pos > FRencoderPosition) {
            pos = FRencoderPosition;
        }
        if (pos > (-1 * FLencoderPosition)) {
            pos = (-1 * FLencoderPosition);
        }

        return pos - prevPos;
    }

    public void updatePosition() {

        FieldPose.updatePosition();

        if (FieldPose.isValidPosition) {
            v3PhotonVision.x = FieldPose.position.getX();
            v3PhotonVision.y = FieldPose.position.getY();

            // Average gyro correction over 10 samples
            gyroCorrection *= 0.9;
            gyroCorrection += ((FieldPose.getRotationDegrees() - getRawHeading()) / 10.0);

            SmartDashboard.putNumber("PhotonVision_X: ", v3PhotonVision.x);
            SmartDashboard.putNumber("PhotonVision_Y: ", v3PhotonVision.y);
            SmartDashboard.putNumber("RawGyro: ", Gyroscope.GetYaw());
            SmartDashboard.putNumber("getRawHeading: ", getRawHeading());
            SmartDashboard.putNumber("FieldPose.getRotationDegrees: ", FieldPose.getRotationDegrees());
            SmartDashboard.putNumber("gyroCorrection: ", gyroCorrection);
            SmartDashboard.putNumber("EncoderPosition: ", BaseDrivetrain.flDriveX.getEncoder().getPosition());
        }

        if (isNewPositionFix()) {
            posFixErrorCorrection = getDistanceTo(v3PhotonVision);
            setPositionFix();

            prevPosition = getEncoderDistance(prevPosition);

        } else {
            // PhotonVision did not find any Apriltag
            posFixErrorCorrection = 0.0;

            // Extrapolate current position from previous position
            heading = getHeading();
            distance = getEncoderDistance(prevPosition);
            prevPosition = distance;

            v3PrevPosition.copy(v3Position);

            v3Position.x += distance * Math.sin(heading);
            v3Position.y += distance * Math.cos(heading);

            if (distance != 0.0) { // No point in filling the log with duplicate data, timestanp will show when
                                   // motionless

                try {
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
                } catch (Exception e) {

                }
            }
        }
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("CurrentPos_X: ", v3Position.x);
        SmartDashboard.putNumber("CurrentPos_Y: ", v3Position.y);
        SmartDashboard.putNumber("Heading: ", heading);
        SmartDashboard.putNumber("Speed: ", speed);
    }
}

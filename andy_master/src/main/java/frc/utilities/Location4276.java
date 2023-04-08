package frc.utilities;

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
    private static long positionUpdateTimeMillisecs;

    public Location4276() {

        v3PhotonVision = new Vector3(0.0, 0.0, 0.0);
        v3PreviousPhotonVision = new Vector3(0.0, 0.0, 0.0);
        v3Position = new Vector3(0.0, 0.0, 0.0);
        v3PrevPosition = new Vector3(0.0, 0.0, 0.0);
    }

    public boolean isNewPositionFix() {
        return (!v3PhotonVision.isEqual(v3PreviousPhotonVision));
    }

    public void setPositionFix() {
        v3PreviousPhotonVision.copy(v3PhotonVision);
        v3Position.copy(v3PhotonVision);
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

        FieldPose.updatePosition();
        SmartDashboard.putNumber("robotPose_X: ", FieldPose.position.getX());
        SmartDashboard.putNumber("robotPose_Y: ", FieldPose.position.getY());

        if(FieldPose.isValidPosition) { 
            v3PhotonVision.x = FieldPose.position.getX();
            v3PhotonVision.y = FieldPose.position.getY();
        }
  
        if (isNewPositionFix()) {
            posFixErrorCorrection = getDistanceTo(v3PhotonVision);
            setPositionFix();

        } else {
            // PhotonVision did not find any Apriltag
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
            
            try{// of stillnes
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
            catch (Exception e){
                
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

package frc.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.systems.BaseDrivetrain;

public class Location4276 {

    // Theory of operation
    //
    // Driver Station and Robot share the following NetworkTable
    // Driver station gets AprilTag info directly from the limelight
    // If AprilTag(s) found
    // isValidPositionFix = true
    // Driver station sets position and velocity table entries
    // Driver station compares robot pose to raw gyro and sets gyro offset
    // Else
    // isValidPositionFix = false
    // Robot uses corrected gyro heading and encoder based speed to update position
    // and velocity table entries
    // Endif
    //
    // At start of Autonomous period, requestControlOfRobotFromDriverStation is set
    // true, (and back to false at end of auto, or if the joysticks are moved)
    // while requestControlOfRobotFromDriverStation == true
    // Driver station updates DESIRED velocity table entries depending on reported
    // position, even if AprilTags are not visible
    // Robot will try to match speed and heading to the desired velocity table
    // entries, probably many times between updates from the driver station

    private static NetworkTable tablePosition;
    private static NetworkTableEntry isValidPositionFix;
    private static NetworkTableEntry requestControlOfRobotFromDriverStation;
    private static NetworkTableEntry pos_X;
    private static NetworkTableEntry pos_Y;
    private static NetworkTableEntry pos_Z;
    private static NetworkTableEntry vel_X;
    private static NetworkTableEntry vel_Y;
    private static NetworkTableEntry vel_Z;
    private static NetworkTableEntry desiredVel_X;
    private static NetworkTableEntry desiredVel_Y;
    private static NetworkTableEntry desiredVel_Z;
    private static NetworkTableEntry gyroRaw;
    private static NetworkTableEntry gyroOffset;

    private long positionUpdateTimeNanosecs;

    public Location4276() {
        tablePosition = NetworkTableInstance.getDefault().getTable("pos4276");
        isValidPositionFix = tablePosition.getEntry("isvalidpositionfix");
        pos_X = tablePosition.getEntry("posx");
        pos_Y = tablePosition.getEntry("posy");
        pos_Z = tablePosition.getEntry("posz");
        vel_X = tablePosition.getEntry("velx");
        vel_Y = tablePosition.getEntry("vely");
        vel_Z = tablePosition.getEntry("velz");
        requestControlOfRobotFromDriverStation = tablePosition.getEntry("requestcontrol");
        desiredVel_X = tablePosition.getEntry("desiredvelx");
        desiredVel_Y = tablePosition.getEntry("desiredvely");
        desiredVel_Z = tablePosition.getEntry("desiredvelz");
        gyroRaw = tablePosition.getEntry("gyroraw");
        gyroOffset = tablePosition.getEntry("gyrooffset");

        positionUpdateTimeNanosecs = java.lang.System.nanoTime();
    }

    public boolean getrequestControlOfRobotFromDriverStation() {
        return vel_Z.getBoolean(false);
    }

    public void setrequestControlOfRobotFromDriverStation(boolean val) {
        requestControlOfRobotFromDriverStation.setBoolean(val);
    }

    public boolean getIsValidPositionFix() {
        return isValidPositionFix.getBoolean(false);
    }

    public void setIsValidPositionFix(boolean val) {
        isValidPositionFix.setBoolean(val);
    }

    public double getPos_X() {
        return pos_X.getDouble(0.0);
    }

    public void setPos_X(double val) {
        pos_X.setDouble(val);
    }

    public double getPos_Y() {
        return pos_Y.getDouble(0.0);
    }

    public void setPos_Y(double val) {
        pos_Y.setDouble(val);
    }

    public double getPos_Z() {
        return pos_Z.getDouble(0.0);
    }

    public void setPos_Z(double val) {
        pos_Z.setDouble(val);
    }

    public double getVel_X() {
        return vel_X.getDouble(0.0);
    }

    public void setVel_X(double val) {
        vel_X.setDouble(val);
    }

    public double getVel_Y() {
        return vel_Y.getDouble(0.0);
    }

    public void setVel_Y(double val) {
        vel_Y.setDouble(val);
    }

    public double getVel_Z() {
        return vel_Z.getDouble(0.0);
    }

    public void setVel_Z(double val) {
        vel_Z.setDouble(val);
    }

    public double getDesiredVel_X() {
        return desiredVel_X.getDouble(0.0);
    }

    public void setDesiredVel_X(double val) {
        desiredVel_X.setDouble(val);
    }

    public double getDesiredVel_Y() {
        return desiredVel_Y.getDouble(0.0);
    }

    public void setDesiredVel_Y(double val) {
        desiredVel_Y.setDouble(val);
    }

    public double getDesiredVel_Z() {
        return desiredVel_Z.getDouble(0.0);
    }

    public void setDesiredVel_Z(double val) {
        desiredVel_Z.setDouble(val);
    }

    public double getGyroRaw() {
        return gyroRaw.getDouble(0.0);
    }

    public void setGyroRaw(double val) {
        gyroRaw.setDouble(val);
    }

    public double getGyroOffset() {
        return gyroOffset.getDouble(0.0);
    }

    public void setGyroOffset(double val) {
        gyroOffset.setDouble(val);
    }

    public double getDesiredHeading() {
        return Math.atan2(getDesiredVel_X(), getDesiredVel_Y());
    }

    public double getDesiredSpeed() {
        return Math.sqrt((getDesiredVel_X() * getDesiredVel_X()) + (getDesiredVel_Y() * getDesiredVel_Y()));
    }

    public double getGyroHeadingError() {
        double heading = Gyroscope.GetYaw() - getGyroOffset();
        return heading - getDesiredHeading();
    }

    public double getEncoderSpeed() {
        // Find the lowest speed encoder (assume faster speed indicates slippage or
        // minor difference due to turning)
        double speed = BaseDrivetrain.FR_encoder.getVelocity();
        if (speed > BaseDrivetrain.FL_encoder.getVelocity()) {
            speed = BaseDrivetrain.FL_encoder.getVelocity();
        } else if (speed > BaseDrivetrain.BR_encoder.getVelocity()) {
            speed = BaseDrivetrain.BR_encoder.getVelocity();
        } else if (speed > BaseDrivetrain.BL_encoder.getVelocity()) {
            speed = BaseDrivetrain.BL_encoder.getVelocity();
        }
        return getDesiredSpeed() - speed;
    }

    public double getEncoderSpeedError() {
        return getDesiredSpeed() - getEncoderSpeed();
    }

    public void updatePosition() {

        if (!getIsValidPositionFix()) {
            // Limelight did not find any Apriltag
            // Estimate current heading and speed to extrapolate current position from
            // previous position
            double prev_X = getPos_X();
            double prev_Y = getPos_Y();

            double heading = getGyroRaw() + getGyroOffset();

            long prevTicks = positionUpdateTimeNanosecs;
            positionUpdateTimeNanosecs = java.lang.System.nanoTime();
            long deltaTimeNanosecs = (positionUpdateTimeNanosecs - prevTicks);

            double speed = getEncoderSpeed();
            double distance = speed * deltaTimeNanosecs;

            double distance_X = distance * Math.cos(heading);
            double distance_Y = distance * Math.sin(heading);

            setPos_X(prev_X + distance_X);
            setPos_Y(prev_Y + distance_Y);
        } // else if AprilTags found the driver station is continuously updating this
          // position via NetworkTables
    }
}

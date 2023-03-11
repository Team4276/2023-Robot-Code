package frc.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // These items go from RoboRio to driver station
    private static NetworkTableEntry requestControlOfRobotFromDriverStation;
    private static NetworkTableEntry pos_X;
    private static NetworkTableEntry pos_Y;
    private static NetworkTableEntry pos_Z;
    private static NetworkTableEntry vel_X;
    private static NetworkTableEntry vel_Y;
    private static NetworkTableEntry vel_Z;
    private static NetworkTableEntry gyroRaw;

    // These items go from driver station to RoboRio
    private static NetworkTableEntry DS_posfix_x;
    private static NetworkTableEntry DS_posfix_y;
    private static NetworkTableEntry DS_posfix_z;
    private static NetworkTableEntry DS_velfix_x;
    private static NetworkTableEntry DS_velfix_y;
    private static NetworkTableEntry DS_velfix_z;
    private static NetworkTableEntry DS_desiredVel_x;
    private static NetworkTableEntry DS_desiredVel_y;
    private static NetworkTableEntry DS_desiredVel_z;
    private static NetworkTableEntry DS_gyroOffset;

    private static double prev_dsposfix_x = 0.0;
    private static double prev_dsposfix_y = 0.0;
    private static double prev_dsposfix_z = 0.0;
    private static double prev_dsvelfix_x = 0.0;
    private static double prev_dsvelfix_y = 0.0;
    private static double prev_dsvelfix_z = 0.0;

    private long positionUpdateTimeNanosecs;

    public Location4276() {
        tablePosition = NetworkTableInstance.getDefault().getTable("pos4276");

        requestControlOfRobotFromDriverStation = tablePosition.getEntry("requestcontrol");
        pos_X = tablePosition.getEntry("posx");
        pos_Y = tablePosition.getEntry("posy");
        pos_Z = tablePosition.getEntry("posz");
        vel_X = tablePosition.getEntry("velx");
        vel_Y = tablePosition.getEntry("vely");
        vel_Z = tablePosition.getEntry("velz");
        gyroRaw = tablePosition.getEntry("gyroraw");

        DS_posfix_x = tablePosition.getEntry("dsposfixx");
        DS_posfix_y = tablePosition.getEntry("dsposfixy");
        DS_posfix_z = tablePosition.getEntry("dsposfixz");
        DS_velfix_x = tablePosition.getEntry("dsvelfixx");
        DS_velfix_y = tablePosition.getEntry("dsvelfixy");
        DS_velfix_z = tablePosition.getEntry("dsvelfixz");
        DS_desiredVel_x = tablePosition.getEntry("dsdesiredvelx");
        DS_desiredVel_y = tablePosition.getEntry("dsdesiredvely");
        DS_desiredVel_z = tablePosition.getEntry("dsdesiredvelz");
        DS_gyroOffset = tablePosition.getEntry("dsgyrooffset");

        positionUpdateTimeNanosecs = java.lang.System.nanoTime();
    }

    public boolean isNewPositionFix() {
        return ((prev_dsposfix_x != DS_posfix_x.getDouble(0.0))
                || (prev_dsposfix_y != DS_posfix_y.getDouble(0.0))
                || (prev_dsposfix_z != DS_posfix_z.getDouble(0.0))
                || (prev_dsvelfix_x != DS_velfix_x.getDouble(0.0))
                || (prev_dsvelfix_y != DS_velfix_y.getDouble(0.0))
                || (prev_dsvelfix_z != DS_velfix_z.getDouble(0.0)));
    }

    public void setPositionFix() {
        prev_dsposfix_x = DS_posfix_x.getDouble(0.0);
        prev_dsposfix_y = DS_posfix_y.getDouble(0.0);
        prev_dsposfix_z = DS_posfix_z.getDouble(0.0);
        prev_dsvelfix_x = DS_velfix_x.getDouble(0.0);
        prev_dsvelfix_y = DS_velfix_y.getDouble(0.0);
        prev_dsvelfix_z = DS_velfix_z.getDouble(0.0);

        pos_X.setDouble(prev_dsposfix_x);
        pos_Y.setDouble(prev_dsposfix_y);
        pos_Z.setDouble(prev_dsposfix_z);
        vel_X.setDouble(prev_dsvelfix_x);
        vel_Y.setDouble(prev_dsvelfix_y);
        vel_Z.setDouble(prev_dsvelfix_z);
    }

    public boolean getrequestControlOfRobotFromDriverStation() {
        return requestControlOfRobotFromDriverStation.getBoolean(false);
    }

    public void setrequestControlOfRobotFromDriverStation(boolean val) {
        requestControlOfRobotFromDriverStation.setBoolean(val);
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

    public double getGyroRaw() {
        return gyroRaw.getDouble(0.0);
    }

    public void setGyroRaw(double val) {
        gyroRaw.setDouble(val);
    }

    public double getDsDesiredVel_x() {
        return DS_desiredVel_x.getDouble(0.0);
    }

    public void setDsDesiredVel_x(double val) {
        DS_desiredVel_x.setDouble(val);
    }

    public double getDsDesiredVel_y() {
        return DS_desiredVel_y.getDouble(0.0);
    }

    public void setDsDesiredVel_y(double val) {
        DS_desiredVel_y.setDouble(val);
    }

    public double getDsDesiredVel_z() {
        return DS_desiredVel_z.getDouble(0.0);
    }

    public void setDsDesiredVel_z(double val) {
        DS_desiredVel_z.setDouble(val);
    }

    public double getDsGyroOffset() {
        return DS_gyroOffset.getDouble(0.0);
    }

    public void setDsGyroOffset(double val) {
        DS_gyroOffset.setDouble(val);
    }

    public double getDsDesiredHeading() {
        return Math.atan2(getDsDesiredVel_x(), getDsDesiredVel_y());
    }

    public double getDsDesiredSpeed() {
        return Math.sqrt((getDsDesiredVel_x() * getDsDesiredVel_x()) + (getDsDesiredVel_y() * getDsDesiredVel_y()));
    }

    public double getGyroHeading() {
        return Gyroscope.GetYaw() - getDsGyroOffset();
    }

    public double getGyroHeadingError() {
        return getGyroHeading() - getDsDesiredHeading();
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

    public double getEncoderSpeedError() {
        return getDsDesiredSpeed() - getEncoderSpeed();
    }

    public void updatePosition() {

        if (isNewPositionFix()) {
            setPositionFix();
        } else {
            // Limelight did not find any Apriltag
            // Estimate current heading and speed to extrapolate current position from
            // previous position
            double prev_X = getPos_X();
            double prev_Y = getPos_Y();

            double heading = getGyroRaw() + getDsGyroOffset();

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

        SmartDashboard.putBoolean("requestControlOfRobotFromDriverStation ",requestControlOfRobotFromDriverStation.getBoolean(false));
        SmartDashboard.putNumber("pos_X ", pos_X.getDouble(0.0));
        SmartDashboard.putNumber("pos_Y ", pos_Y.getDouble(0.0));
        SmartDashboard.putNumber("pos_Z ", pos_Z.getDouble(0.0));

        SmartDashboard.putNumber("gyroRaw ", gyroRaw.getDouble(0.0));
        SmartDashboard.putNumber("speed", getEncoderSpeed());
    }
}

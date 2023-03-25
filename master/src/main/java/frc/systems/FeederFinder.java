package frc.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.LogJoystick;
import frc.utilities.Vector3;

public class FeederFinder {

    private static final int idApriltagOnHumanPlayerBoard = 14;

    private static final double stopRangeFeet = 1.5; // Distance from camera to the front of the bumper
    private static final double slowRangeFeet = stopRangeFeet + 1.0;
    private static final double medRangeFeet = stopRangeFeet + 2.0;

    private static final double fastPower = 0.9;
    private static final double medPower = 0.6;
    private static final double slowPower = 0.2;

    private static final double lineFollowerDeadZone = 0.1;
    private static final double lineFollowerSlowZone = 0.1;

    private static boolean isValidApriltagPosition = false;
    private static Vector3 v3ApriltagPositionInRobotRelativeCoordinates;

    public FeederFinder() {
        v3ApriltagPositionInRobotRelativeCoordinates = new Vector3(0.0, 0.0, 0.0);
    }

    public static void updatePeriodic() {

        if (Robot.rightJoystick.getRawButton(LogJoystick.B7)) {

            isValidApriltagPosition = checkLimelightForHumanPLayerBoard();
            double power_R = 0.0;
            double power_L = 0.0;
            if (isValidApriltagPosition) {

                // Units are feet
                // X toward the robot's right (Right Vector)
                // Y Pointing forward (Forward Vector)
                // Z Pointing upward (Up Vector)

                // Drive robot forward, steering to make the "Y" coordinate zero so we are going
                // straght toward
                // the Apriltag on the human player board
                SmartDashboard.putNumber("APriltag14_X", v3ApriltagPositionInRobotRelativeCoordinates.x);
                SmartDashboard.putNumber("APriltag14_Y", v3ApriltagPositionInRobotRelativeCoordinates.y);
                if (v3ApriltagPositionInRobotRelativeCoordinates.y > stopRangeFeet) {
                    power_R = slowPower;
                    power_L = slowPower;
                    if (v3ApriltagPositionInRobotRelativeCoordinates.y > slowRangeFeet) {
                        power_R = medPower;
                        power_L = medPower;
                    } else if (v3ApriltagPositionInRobotRelativeCoordinates.y > medRangeFeet) {
                        power_R = fastPower;
                        power_L = fastPower;
                    }

                    if (v3ApriltagPositionInRobotRelativeCoordinates.x > lineFollowerDeadZone) {
                        power_R += slowPower;
                    } else if (v3ApriltagPositionInRobotRelativeCoordinates.x > lineFollowerSlowZone) {
                        power_R += medPower;
                    }

                    if (v3ApriltagPositionInRobotRelativeCoordinates.x < (-1 * lineFollowerDeadZone)) {
                        power_L += slowPower;
                    } else if (v3ApriltagPositionInRobotRelativeCoordinates.x < (-1 * lineFollowerSlowZone)) {
                        power_L += medPower;
                    }
                }
            } else {
                // It is expected that the Limelight wil lose sight of the Apriltag as it gets
                // close to the feeder
                // If the driver continues to hold the button down drive straight forward slowly
                // until the button is released
                power_R = slowPower;
                power_L = slowPower;
            }
            power_R *= -1.0;

            TeleopDrivetrain.assignMotorPower(power_R, power_L);
        }
    }

    public static boolean checkLimelightForHumanPLayerBoard() {

        final double feet_per_meter = 3.28084;

        double[] defaultValue = new double[6];
        double[] positionLimelight = new double[6];
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        long idPrimaryApriltagInView = Robot.ntLimelight.getEntry("tid").getInteger(0);
        if (idPrimaryApriltagInView == idApriltagOnHumanPlayerBoard) {
            positionLimelight = Robot.ntLimelight.getEntry("targetpose_cameraspace")
                    .getDoubleArray(defaultValue); // "targetpose_cameraspace")
            x = positionLimelight[0];
            y = positionLimelight[1];
            z = positionLimelight[2];

            // position units are meters at this point - convert to feet
            //x *= feet_per_meter;
            //y *= feet_per_meter;
            //z *= feet_per_meter;

            // Limelight is offset to the right of the robot centerline
            x -= 0.5; // TODO measure on the robot Units are feet

            v3ApriltagPositionInRobotRelativeCoordinates.set(x, y, z);
            return true;
        }
        return false;
    }

}

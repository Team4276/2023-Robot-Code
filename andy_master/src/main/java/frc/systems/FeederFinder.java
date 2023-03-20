package frc.systems;

import frc.robot.Robot;
import frc.utilities.Vector3;
import frc.utilities.Xbox;

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

        if (Robot.xboxController.getRawButton(Xbox.A)) {

            isValidApriltagPosition = checkLimelightForHumanPLayerBoard();
            if (isValidApriltagPosition) {

                // Units are feet
                // X Pointing forward (Forward Vector)
                // Y Pointing toward the robot's right (Right Vector)
                // Z Pointing upward (Up Vector)

                // Drive robot forward, steering to make the "Y" coordinate zero so we are going
                // straght toward
                // the Apriltag on the human player board
                double power_R = 0.0;
                double power_L = 0.0;
                if (v3ApriltagPositionInRobotRelativeCoordinates.x > stopRangeFeet) {
                    power_R = slowPower;
                    power_L = slowPower;
                    if (v3ApriltagPositionInRobotRelativeCoordinates.x > slowRangeFeet) {
                        power_R = medPower;
                        power_L = medPower;
                    } else if (v3ApriltagPositionInRobotRelativeCoordinates.x > medRangeFeet) {
                        power_R = fastPower;
                        power_L = fastPower;
                    }

                    if (v3ApriltagPositionInRobotRelativeCoordinates.y > lineFollowerDeadZone) {
                        power_R += slowPower;
                    } else if (v3ApriltagPositionInRobotRelativeCoordinates.y > lineFollowerSlowZone) {
                        power_R += medPower;
                    }

                    if (v3ApriltagPositionInRobotRelativeCoordinates.y < (-1 * lineFollowerDeadZone)) {
                        power_L += slowPower;
                    } else if (v3ApriltagPositionInRobotRelativeCoordinates.y < (-1 * lineFollowerSlowZone)) {
                        power_L += medPower;
                    }
                }
                TeleopDrivetrain.assignMotorPower(power_R, power_L);
            }
        }
    }

    public static boolean checkLimelightForHumanPLayerBoard() {

        final double feet_per_meter = 3.28084;

        long idPrimaryApriltagInView = Robot.ntLimelight.getEntry("tid").getInteger(0);
        if (idPrimaryApriltagInView == idApriltagOnHumanPlayerBoard) {
            double[] errorhandle = new double[6];
            double[] positionLimelight = Robot.ntLimelight.getEntry("targetpose_robotspace")
                    .getDoubleArray(errorhandle);
            double x = positionLimelight[0];
            double y = positionLimelight[1];
            double z = positionLimelight[2];

            // position units are meters at this point - convert to feet
            x *= feet_per_meter;
            y *= feet_per_meter;
            z *= feet_per_meter;

            v3ApriltagPositionInRobotRelativeCoordinates.set(x, y, z);
            return true;
        }
        return false;
    }

}

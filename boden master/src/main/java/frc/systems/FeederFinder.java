package frc.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.LogJoystick;
import frc.utilities.Vector3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class FeederFinder {

    private static final int idApriltagOnHumanPlayerBoard = 14;

    private static final double medPower = 0.2;
    private static final double slowPower = 0.15;

    private static final double lineFollowerDeadZone = 0.05;

    private static boolean isValidApriltagPosition = false;
    private static Vector3 v3ApriltagPositionInRobotRelativeCoordinates;


    public FeederFinder() {
        v3ApriltagPositionInRobotRelativeCoordinates = new Vector3(0.0, 0.0, 0.0);
    }

    public static void updatePeriodic() {

        if (Robot.rightJoystick.getRawButton(LogJoystick.B9)) {

            isValidApriltagPosition = checkLimelightForHumanPLayerBoard();
            double power_R = 0.0;
            double power_L = 0.0;
            double default1 = 0;
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

                power_R = medPower;
                power_L = medPower;

                if (v3ApriltagPositionInRobotRelativeCoordinates.x < lineFollowerDeadZone) {
                    power_R += slowPower;
                } else if (v3ApriltagPositionInRobotRelativeCoordinates.x < (-1 * lineFollowerDeadZone)) {
                    power_L += slowPower;
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

        Vector3 _2dapriltaglocation = new Vector3 (0.0, 0.0, 0.0);


        long idPrimaryApriltagInView = Robot.ntLimelight.getEntry("tid").getInteger(0);
        if (idPrimaryApriltagInView == idApriltagOnHumanPlayerBoard) {
            // "targetpose_cameraspace")
            double default1 = 0;
            _2dapriltaglocation.x = Robot.ntLimelight.getEntry("tx").getDouble(_2dapriltaglocation.x);
            _2dapriltaglocation.y = Robot.ntLimelight.getEntry("ty").getDouble(_2dapriltaglocation.y);


            // position units are meters at this point - convert to feet
            _2dapriltaglocation.x *= feet_per_meter;
            _2dapriltaglocation.y *= feet_per_meter;

            v3ApriltagPositionInRobotRelativeCoordinates.set(_2dapriltaglocation.x, _2dapriltaglocation.y, _2dapriltaglocation.z);
            return true;
        }
        return false;
    }

}

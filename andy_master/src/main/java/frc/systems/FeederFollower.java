package frc.systems;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.utilities.LogJoystick;
import frc.utilities.Route;
import frc.utilities.SoftwareTimer;
import frc.utilities.RobotMode.ROBOT_MODE;
import frc.utilities.RobotMode;

public class FeederFollower {

    private Route routeBlueFeeder;
    private Route routeRedFeeder;

    private final double barrierDistanceFromCenterlineInMeters = 1.5616;
    private final double inchesPerMeter = 39.3701;
    private final double robotTrackWidthInMeters = 19.75 / inchesPerMeter;

    private RamseteController pathController;
    private Trajectory pathTrajectory;
    private Trajectory.State goal;
    private ChassisSpeeds adjustedSpeeds;
    private Pose2d currentRobotPose;
    private Timer pathTimer;
    private DifferentialDriveKinematics robotKinematics;

    public FeederFollower() {
        routeBlueFeeder = new Route("paths/BlueFeeder.wpilib.json");
        routeRedFeeder = new Route("paths/RedFeeder.wpilib.json");

        pathController = new RamseteController();
        pathTimer = new Timer();
        robotKinematics = new DifferentialDriveKinematics(robotTrackWidthInMeters);
    }

    public void updatePeriodic() {

        currentRobotPose = Robot.myLocation.getRobotPosition();

        if (Robot.rightJoystick.getRawButton(LogJoystick.B8)) {

            // Choose red or blue feeder on the same side of the field as the robot
            RobotMode.set(ROBOT_MODE.TELEOP_APPROACHING_PATH_START);
            if (currentRobotPose.getX() > 0) {
                pathTrajectory = routeBlueFeeder.trajectory;
                if ((currentRobotPose.getX() > pathTrajectory.getInitialPose().getX())
                        && (currentRobotPose.getY() > barrierDistanceFromCenterlineInMeters)) {
                    if (RobotMode.get() == ROBOT_MODE.TELEOP_APPROACHING_PATH_START) {
                        pathTimer.start();
                    }             
                    RobotMode.set(ROBOT_MODE.TELEOP_FOLLOWING_PATH);
                }
            } else {
                pathTrajectory = routeRedFeeder.trajectory;
                if ((currentRobotPose.getX() < pathTrajectory.getInitialPose().getX())
                        && (currentRobotPose.getY() > barrierDistanceFromCenterlineInMeters)) {
                    if (RobotMode.get() == ROBOT_MODE.TELEOP_APPROACHING_PATH_START) {
                        pathTimer.start();
                    }
                    RobotMode.set(ROBOT_MODE.TELEOP_FOLLOWING_PATH);
                }
            }

            if (RobotMode.get() == ROBOT_MODE.TELEOP_APPROACHING_PATH_START) {
                // Drive straight towords the start of the path
                goal = pathTrajectory.getStates().get(0);
            } else {
                goal = pathTrajectory.sample(pathTimer.get());
            }
            adjustedSpeeds = pathController.calculate(currentRobotPose, goal);

            DifferentialDriveWheelSpeeds wheelSpeeds = robotKinematics.toWheelSpeeds(adjustedSpeeds);
            PIDDrivetrain.setSpeeds(wheelSpeeds);
        }
    }

}
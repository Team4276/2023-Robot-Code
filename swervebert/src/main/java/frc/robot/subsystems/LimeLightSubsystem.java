package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PositionConstants;

public class LimeLightSubsystem extends SubsystemBase {
    public static enum target{
        SCORE_CONE_HIGH,
        SCORE_CONE_MID,
        SCORE_CONE_LOW,
        SCORE_CUBE_HIGH,
        SCORE_CUBE_MID,
        SCORE_CUBE_LOW,
        INTAKE_CONE_GROUND,
        INTAKE_CONE_FEEDER,
        INTAKE_CONE_DOUBLE,
        INTAKE_CUBE_GROUND,
        INTAKE_CUBE_FEEDER,
        INTAKE_CUBE_DOUBLE,
    }

    private Pose2d getTargetInfo(target target){
        switch(target.ordinal()){
            case 0:
                return PositionConstants.scoreConeHigh;
            case 1:
                return PositionConstants.scoreConeHigh;
            case 2:
                return PositionConstants.scoreConeHigh;
            case 3:
                return PositionConstants.scoreConeHigh;
            case 4:
                return PositionConstants.scoreConeHigh;
            case 5:
                return PositionConstants.scoreConeHigh;
            case 6:
                return PositionConstants.scoreConeHigh;
            case 7:
                return PositionConstants.scoreConeHigh;
            case 8:
                return PositionConstants.scoreConeHigh;
            case 9:
                return PositionConstants.scoreConeHigh;
            case 10:
                return PositionConstants.scoreConeHigh;
            case 11:
                return PositionConstants.scoreConeHigh;

            default: break;
        }

        return null;

    }

    /** Placeholder subsystem */
    public LimeLightSubsystem(){
        //weee
    }


    /** Placeholder function */
    public boolean checkIfAligned(){
        return true;
    }

    /** Returns a Pose2d of target where the robot is at the origin. */
    public Translation2d getRelativeTargetPose(target target){
        return null;
    }

    private Trajectory getTargetTrajectory(target target){
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

            // An example trajectory to follow. All units in meters.
            Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(getRelativeTargetPose(target)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

            return exampleTrajectory;


    }

    /** Placeholder function */
    public Trajectory aligne(target target){
        Pose2d ideal = getTargetInfo(target);

        if(ideal == null){
            return null;
        } else {
            return getTargetTrajectory(target);

        }
    }

    
}

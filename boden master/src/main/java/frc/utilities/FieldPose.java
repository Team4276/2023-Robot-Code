
package frc.utilities;

import java.io.IOException;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import frc.systems.BaseDrivetrain;
import frc.systems.PIDDrivetrain;

 public class FieldPose extends BaseDrivetrain {

    public FieldPose(int FLport, int BLport, int FRport, int BRport) {
        super(FLport, BLport, FRport, BRport);
        //TODO Auto-generated constructor stub
    }

  

    private static PhotonCamera cam;

    public static Pose3d Localcopy;
    public static Pose2d Localcopy2d;
    public static Double LocalTimestamp;

    public static DifferentialDriveKinematics m_kinematics =
  new DifferentialDriveKinematics(Units.inchesToMeters(18.85));

    Optional<EstimatedRobotPose> pose;

    public static double[] LeftAndRightWheelVelocity(){
        double[] LeftRight = new double[]{frDriveX.getEncoder().getVelocity(), flDriveX.getEncoder().getVelocity()};        
        return LeftRight;
    }
//method to create a PhotonPoseEstimator object
    public static PhotonPoseEstimator visionEstamate() throws IOException{
        PhotonCamera.setVersionCheckEnabled(false);
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        cam = new PhotonCamera("Arducam_12MP");
        cam.isConnected();
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.5, 0.5), new Rotation3d(1,1,1));
        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);
        return photonPoseEstimator;
    }
    
    public static Pose2d ExampleGlobalMeasurementSensor() throws NoSuchElementException {
        double[] copyLeftAndRight = LeftAndRightWheelVelocity();
        DifferentialDriveWheelSpeeds wheelspeeds = new DifferentialDriveWheelSpeeds(copyLeftAndRight[0], copyLeftAndRight[1]);
        ChassisSpeeds robotspeeds = m_kinematics.toChassisSpeeds(wheelspeeds);
        PhotonPoseEstimator currentEstimator;
        try {
            currentEstimator = visionEstamate();
            Optional<EstimatedRobotPose> EstimatorInfo = currentEstimator.update();
            Localcopy = EstimatorInfo.get().estimatedPose;
            LocalTimestamp = EstimatorInfo.get().timestampSeconds;
            Localcopy2d = new Pose2d(Localcopy.getX(), Localcopy.getY(), Localcopy.getRotation().toRotation2d());
             
        } catch (IOException e) {
            DriverStation.reportWarning("IOException caught, Field file could not be found pathfinding will not be functinal", false);
            }
        
        
        // Utility class
        
        final DifferentialDrivePoseEstimator m_poseEstimator =
        new DifferentialDrivePoseEstimator(
            m_kinematics,
            Gyroscope.getRotation2d(),
            frDriveX.getEncoder().getPosition(),
            flDriveX.getEncoder().getPosition(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
            );
           
            m_poseEstimator.addVisionMeasurement(Localcopy2d, LocalTimestamp);
            return m_poseEstimator.getEstimatedPosition();
    }
}



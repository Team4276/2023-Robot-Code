
package frc.utilities;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldPose {

    public static boolean isValidPosition;
    public static Pose2d position;

    private static PhotonCamera cam;
    private static PhotonPoseEstimator photonPoseEstimator;

    public FieldPose() {
        cam = new PhotonCamera("HD_Pro_Webcam_C920");
        position = new Pose2d();

        try {
            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    cam, robotToCam);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public static double getRotationDegrees() {
        return position.getRotation().getDegrees();
    }

    public static void updatePosition() {

        photonPoseEstimator.setReferencePose(position);

        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        isValidPosition = pose.isPresent();
        if (isValidPosition) {
            position = pose.get().estimatedPose.toPose2d();
        }
    }

}

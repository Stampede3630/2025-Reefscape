package frc.robot.subsystems;

import java.util.List;
import java.util.function.Function;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class NewPhotonCamera extends PhotonCamera{
    private final Transform3d camToRobot;
    private final PhotonPoseEstimator photonPoseEstimator;
    private List<PhotonTrackedTarget> m_visibleTags;

    public NewPhotonCamera(String cameraName, Transform3d camToRobot, Function<EstimatedRobotPose, Matrix<N3, N1>> stdDevFunction) {
        super(cameraName);
        this.camToRobot = camToRobot;
        photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camToRobot);
    }

    public EstimatedRobotPose getEstimatedRobotPose() {
        PhotonPipelineResult r = this.getLatestResult();
        m_visibleTags = r.getTargets();
        return photonPoseEstimator.update(r).orElseGet(() -> {return null;});
    }
}

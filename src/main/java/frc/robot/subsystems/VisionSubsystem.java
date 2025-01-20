package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera m_camera = new PhotonCamera("imx708_wide");
    private final Transform3d m_cameraTransform = new Transform3d(
        new Translation3d(),
        new Rotation3d(0.0, 0.0, 0.0)
    );

    private final AprilTagFieldLayout m_apriltagLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2025Reefscape
    );

    private final PhotonPoseEstimator m_poseEstimator = new PhotonPoseEstimator(
        m_apriltagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_cameraTransform
    );

    private final SwerveDrivePoseEstimator m_drivetrainPoseEstimator;

    public VisionSubsystem(SwerveDrivePoseEstimator drivetrainPoseEstimator) {
        m_drivetrainPoseEstimator = drivetrainPoseEstimator;
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

        // Debug snapshots.
        m_camera.takeInputSnapshot();
        m_camera.takeOutputSnapshot();
    }

    public void periodic() {
        try {
            updateVisionData();
        }

        catch (Exception e) {
            System.out.println(e.getStackTrace());
        }
    }

    public double distanceToTag(int apriltag) {
        var targets = m_camera.getLatestResult().targets;
        for (PhotonTrackedTarget target : targets) {
            if (target.fiducialId == apriltag) {
                return PhotonUtils.calculateDistanceToTargetMeters(
                    0.2667, //10.5 INCHES
                    0.2667,
                    0.0,
                    Units.degreesToRadians(target.getPitch())
                );
            }
        }

        return Double.NaN;
    }

    public PhotonTrackedTarget getTarget(int apriltag) {
        var results = m_camera.getLatestResult();
        for (PhotonTrackedTarget target : results.targets) {
            if (target.fiducialId == apriltag) {
                return target;
            }
        }

        return new PhotonTrackedTarget();
    }

    private void updateVisionData() {
        List<PhotonPipelineResult> cameraResults = m_camera.getAllUnreadResults();

        for (PhotonPipelineResult cameraResult : cameraResults) {
            // TODO: The estimations can be scaled by the distances and velocity of the robot.
            Optional<EstimatedRobotPose> estimatorResults = m_poseEstimator.update(cameraResult);

            if (estimatorResults.isPresent()) {
                Pose2d estimatedPose = estimatorResults.get().estimatedPose.toPose2d();
                //System.out.println("Poses x, y:" + estimatedPose.getX() + " " + estimatedPose.getY());
                m_drivetrainPoseEstimator.addVisionMeasurement(estimatedPose, cameraResult.getTimestampSeconds());
            }
        }
    }
}

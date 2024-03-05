package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraSubsystem extends SubsystemBase {
      protected PhotonCamera m_camera;
      protected PhotonPipelineResult m_result;
      protected PhotonPoseEstimator m_PoseEstimator;
      protected final Transform3d kRobotToLeftCamera;
      protected int m_bestTargetFiducialId;

      public CameraSubsystem (String cameraName, Transform3d robotToCameraTransform) {
        m_camera = new PhotonCamera(cameraName);
        kRobotToLeftCamera = robotToCameraTransform;
      }

      public void configure() {
        m_PoseEstimator = new PhotonPoseEstimator(VisionConstants.APRIL_TAG_FIELD_LAYOUT, VisionConstants.POSE_STRATEGY, m_camera, kRobotToLeftCamera);
      }

      public void updateResult() {
        m_result = m_camera.getLatestResult();
        if (m_result.hasTargets()) {
          m_bestTargetFiducialId = m_result.getBestTarget().getFiducialId();
        }
      }
      public double getTimeStamp(){
        return m_result.getLatencyMillis();
      }

      public EstimatedRobotPose getEstimatedRobotPose() {
        Optional<EstimatedRobotPose> estimatePose = m_PoseEstimator.update(m_result);
        return estimatePose.get();
      }


      public PNPResult getPNPResult(){
        PNPResult PNPEstimate = m_result.getMultiTagResult().estimatedPose;
        return PNPEstimate;
      }


      public Pose3d pose3dFromPNPResult(PNPResult result){
        return new Pose3d(result.best.getTranslation(), result.best.getRotation());
      }
}
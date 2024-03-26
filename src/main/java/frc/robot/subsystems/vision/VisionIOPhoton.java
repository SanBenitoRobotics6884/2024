package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.CameraConstants;

import static frc.robot.Constants.Vision.*;

import java.util.List;
import java.util.Optional;

public class VisionIOPhoton implements VisionIO {
  private PhotonCamera m_camera;
  private PhotonPoseEstimator m_poseEstimator;

  public VisionIOPhoton(CameraConstants cameraConstants) {
    m_camera = new PhotonCamera(cameraConstants.cameraName);
    m_poseEstimator = new PhotonPoseEstimator(LAYOUT,PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera,cameraConstants.robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Optional<EstimatedRobotPose> estimatedPose = m_poseEstimator.update();
    if (estimatedPose.isPresent()) {
      inputs.estimatedPose = estimatedPose.get().estimatedPose;
      inputs.timestampSeconds = estimatedPose.get().timestampSeconds;
    } else {
      inputs.estimatedPose = null;
    }
    List<PhotonTrackedTarget> targets = m_camera.getLatestResult().targets;
    inputs.isConnected = m_camera.isConnected();
    inputs.fiducialIds = new int[targets.size()];
    for (int i = 0; i < targets.size(); i++) {
      inputs.fiducialIds[i] = targets.get(i).getFiducialId();
    }

    if (targets.size() != 0) {
      Logger.recordOutput("vision/yaw", targets.get(0).getYaw());
    }
  }

  @Override
  public void takeSnapshot() {
    m_camera.takeInputSnapshot();
    m_camera.takeOutputSnapshot();
  }
  
}

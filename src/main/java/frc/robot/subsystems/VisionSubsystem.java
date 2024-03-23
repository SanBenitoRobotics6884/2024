// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  
  SwerveSubsystem m_swerveSubsystem;
  PhotonCamera m_camera = 
    new PhotonCamera("Vision | 🤫🧏");
  AprilTagFieldLayout m_fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  Transform3d botToCam = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
  PhotonPoseEstimator m_poseEstimator = 
    new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, botToCam);
  
  int m_fiducialId;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(PhotonCamera camera, SwerveSubsystem swerve) {

    m_camera = camera;
    m_swerveSubsystem = swerve;
    m_fiducialId = 0;

    m_camera.setDriverMode(false);
    m_camera.getLatestResult().getLatencyMillis();

    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_camera.getLatestResult().hasTargets()) {
      var results = m_camera.getLatestResult();
      PhotonTrackedTarget bestTarget = results.getBestTarget();
      m_fiducialId = bestTarget.getFiducialId();
    }
    var pose = m_poseEstimator.update();
    
  }
}

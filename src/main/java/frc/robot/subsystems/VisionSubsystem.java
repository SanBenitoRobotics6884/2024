// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  
  PhotonCamera m_camera = new PhotonCamera("Vision | 🤫🧏");
  AprilTagFieldLayout m_fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  SwerveSubsystem m_swerveSubsystem;
  
  int m_fiducialId;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(PhotonCamera camera, SwerveSubsystem swerve) {
    // i have an idea we'll see if i remember tomorrow
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
      var m_results = m_camera.getLatestResult();
      PhotonTrackedTarget m_bestTarget = m_results.getBestTarget();

      m_fiducialId = m_bestTarget.getFiducialId();
      m_bestTarget.getBestCameraToTarget();
   
    }
  }

  public void fieldRelativePose(PhotonTrackedTarget bestTarget, ) {
    PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), );
  }
}

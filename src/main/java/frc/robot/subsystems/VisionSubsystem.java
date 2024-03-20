// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  
  PhotonCamera m_camera = new PhotonCamera("Vision | Mew and Mogging Edition 🤫🧏");

  SwerveSubsystem m_swerveSubsystem;
  double m_pitch;
  double m_yaw;
  double m_skew;
  int m_fiducialId;
  double m_targetHeight;
  Pose3d m_tagPose3d;
  

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(PhotonCamera camera, SwerveSubsystem swerve) {
    // i have an idea we'll see if i remember tomorrow
    m_camera = camera;
    m_swerveSubsystem = swerve;
    m_yaw = 0.0;
    m_fiducialId = 0;
    m_targetHeight = 0.0;
    m_tagPose3d = new Pose3d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_camera.getLatestResult().hasTargets()) {
      var m_results = m_camera.getLatestResult();
      PhotonTrackedTarget m_bestTarget = m_results.getBestTarget();

      m_fiducialId = m_bestTarget.getFiducialId();
      m_pitch = m_bestTarget.getPitch();
      m_yaw = m_bestTarget.getYaw();
      m_skew = m_bestTarget.getSkew();
      m_bestTarget.getPoseAmbiguity();
      m_bestTarget.getBestCameraToTarget();

      //m_bestTarget.getAlternateCameraToTarget();
      // what in the frink is reprojection and how can it error 😭
      // im still debating how i would use taking snapshots of what the camera sees and why, and even when.
    }
  }
  // fr debating things (i'll ask you tomowowow lucus 👍)
}

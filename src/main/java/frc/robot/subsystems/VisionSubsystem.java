// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionSubsystem.*;

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
            
      var fieldtags = m_fieldLayout.getTagPose(m_fiducialId).get().getRotation().getZ();

    }
    Optional<EstimatedRobotPose> pose = m_poseEstimator.update();
    if (pose.isPresent() 
        && Math.abs(pose.get().estimatedPose.getTranslation().getZ()) < POSE_HEIGHT_THRESHOLD 
        && Math.abs(pose.get().estimatedPose.getRotation().getY()) < POSE_ANGLE_THRESHOLD 
        && Math.abs(pose.get().estimatedPose.getRotation().getX()) < POSE_ANGLE_THRESHOLD ) {
      m_swerveSubsystem.addVisionMeasurement(
          pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
    }
  }


}

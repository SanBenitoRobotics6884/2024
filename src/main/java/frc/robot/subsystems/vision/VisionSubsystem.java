// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.Vision.*;

public class VisionSubsystem extends SubsystemBase {
  VisionIO m_io;
  VisionIOInputsAutoLogged m_inputs = new VisionIOInputsAutoLogged();
  
  SwerveSubsystem m_swerveSubsystem;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerve, VisionIO io) {
    m_swerveSubsystem = swerve;
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("vision", m_inputs);

    if (m_inputs.estimatedPose != null
        && Math.abs(m_inputs.estimatedPose.getTranslation().getZ()) < POSE_HEIGHT_THRESHOLD 
        && Math.abs(m_inputs.estimatedPose.getRotation().getY()) < POSE_ANGLE_THRESHOLD 
        && Math.abs(m_inputs.estimatedPose.getRotation().getX()) < POSE_ANGLE_THRESHOLD ) {
      m_swerveSubsystem.addVisionMeasurement(
          m_inputs.estimatedPose.toPose2d(), m_inputs.timestampSeconds);
    }

    if (m_inputs.estimatedPose != null) {
      Logger.recordOutput("vision/pose", m_inputs.estimatedPose.toPose2d());
    }
  }

  public void takeSnapshot() {
    m_io.takeSnapshot();
  }


}

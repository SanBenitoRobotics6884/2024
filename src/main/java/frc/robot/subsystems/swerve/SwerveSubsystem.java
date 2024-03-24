// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Swerve.*;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

public class SwerveSubsystem extends SubsystemBase {
  private ModuleIO[] m_modules = new ModuleIO[] {
      new ModuleIOSparkMax(
          FR_DRIVE_ID, FR_STEER_ID, FR_ENCODER_ID, 
          FR_DRIVE_INVERTED, FR_STEER_INVERTED, FR_OFFSET_ROTATIONS),
      new ModuleIOSparkMax(
          FL_DRIVE_ID, FL_STEER_ID, FL_ENCODER_ID, 
          FL_DRIVE_INVERTED, FL_STEER_INVERTED, FL_OFFSET_ROTATIONS),
      new ModuleIOSparkMax(
          BR_DRIVE_ID, BR_STEER_ID, BR_ENCODER_ID, 
          BR_DRIVE_INVERTED, BR_STEER_INVERTED, BR_OFFSET_ROTATIONS),
      new ModuleIOSparkMax(
          BL_DRIVE_ID, BL_STEER_ID, BL_ENCODER_ID, 
          BL_DRIVE_INVERTED, BL_STEER_INVERTED, BL_OFFSET_ROTATIONS)};
  private ModuleIOInputsAutoLogged[] m_moduleInputs = new ModuleIOInputsAutoLogged[4];

  private GyroIO m_gyro = new GyroIOPigeon(PIGEON_ID);
  private GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      FR_LOCATION,
      FL_LOCATION,
      BR_LOCATION,
      BL_LOCATION);

  private Pose2d m_pose = new Pose2d();
  private SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4];
  private SwerveModuleState[] m_moduleStates = new SwerveModuleState[4];

  private SwerveDrivePoseEstimator m_odometry;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    for (int i = 0; i < 4; i++) {
      m_modulePositions[i] = new SwerveModulePosition();
      m_moduleStates[i] = new SwerveModuleState();
      m_moduleInputs[i] = new ModuleIOInputsAutoLogged();
    }

    m_odometry = new SwerveDrivePoseEstimator(
        m_kinematics,
        getAngle(),
        m_modulePositions,
        m_pose);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::driveRobotOriented,
        PATH_FOLLOWER_CONFIG,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  @Override
  public void periodic() {
    m_gyro.updateInputs(m_gyroInputs);
    Logger.processInputs("swerve/gyro", m_gyroInputs);
    for (int i = 0; i < 4; i++) {
      m_modules[i].updateInputs(m_moduleInputs[i]);
      Logger.processInputs("swerve/module" + i, m_moduleInputs[i]);
      Logger.recordOutput("swerve/module" + i + "/double-angle", m_moduleInputs[i].relativeAngle.getDegrees());
      m_modulePositions[i].angle = m_moduleInputs[i].relativeAngle;
      m_modulePositions[i].distanceMeters = m_moduleInputs[i].drivePosition;
      m_moduleStates[i].angle = m_moduleInputs[i].relativeAngle;
      m_moduleStates[i].speedMetersPerSecond = m_moduleInputs[i].driveVelocity;
    }
    Logger.recordOutput("swerve/current-states", m_moduleStates);

    m_pose = m_odometry.update(getAngle(), m_modulePositions);
    Logger.recordOutput("swerve/pose", m_pose);
  }

  public void driveRobotOriented(ChassisSpeeds speeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    Logger.recordOutput("swerve/desired-states", states);

    for(int i = 0; i < 4; i++) {
      m_modules[i].setState(SwerveModuleState.optimize(
          states[i], m_moduleInputs[i].relativeAngle));
    }
  }

  public void driveFieldOriented(ChassisSpeeds speeds) {
    driveRobotOriented(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle()));
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_gyroInputs.yaw);
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public void setPose(Pose2d pose) {
    m_odometry.resetPosition(getAngle(), m_modulePositions, pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(m_moduleStates);
  }

  public void zeroYaw() {
    m_gyro.setYaw(0);
  }

  public void setYaw(double angle) {
    m_gyro.setYaw(angle);
  }

  public void seedModuleMeasurements() {
    for (int i = 0; i < 4; i++) {
      m_modules[i].resetToAbsolute();
    }
  }

  public void zeroPose() {
    m_pose = new Pose2d();
    m_odometry.resetPosition(
      new Rotation2d(), 
      m_modulePositions, 
      m_pose);
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    m_odometry.addVisionMeasurement(pose, timestamp);
  }

}

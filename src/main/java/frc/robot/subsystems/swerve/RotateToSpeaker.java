// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;

public class RotateToSpeaker extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private DoubleSupplier m_forward;
  private DoubleSupplier m_strafe;

  private PIDController m_rotationPID;

  /** Creates a new RotateToSpeaker. */
  public RotateToSpeaker(SwerveSubsystem swerveSubsystem, DoubleSupplier forward, DoubleSupplier strafe) {
    m_swerveSubsystem = swerveSubsystem;
    m_forward = forward;
    m_strafe = strafe;

    m_rotationPID = new PIDController(0, 0, 0);
    m_rotationPID.enableContinuousInput(-180, 180);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rotationPID.setSetpoint(Field.getAngleDegreesToSpeaker(m_swerveSubsystem.getPose()));
    m_swerveSubsystem.driveFieldOriented(new ChassisSpeeds(
        m_forward.getAsDouble(), m_strafe.getAsDouble(), 
        m_rotationPID.calculate(m_swerveSubsystem.getAngle().getDegrees())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

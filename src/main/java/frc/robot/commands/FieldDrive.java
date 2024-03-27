// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.annotation.Nulls;
import com.fasterxml.jackson.annotation.JsonFormat.Value;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class FieldDrive extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private DoubleSupplier m_forward;
  private DoubleSupplier m_strafe;
  private DoubleSupplier m_rotation;
  XboxController xboxController = new XboxController(1); 
 Joystick joystick = new Joystick(1); 
  /** Creates a new FieldDrive. */
  public FieldDrive(SwerveSubsystem subsystem, DoubleSupplier forward, 
                    DoubleSupplier strafe, DoubleSupplier rotation) {
    m_swerveSubsystem = subsystem;
    m_forward = forward;
    m_strafe = strafe;
    m_rotation = rotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
     XboxController.setRumble(RumbleType.KBothRumble, 100);
     joystick.setRumble(RumbleType.KBothRumble, 100);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.driveFieldOriented(ChassisSpeeds.discretize(
        m_forward.getAsDouble(), m_strafe.getAsDouble(), m_rotation.getAsDouble(), 0.020));
  }

}

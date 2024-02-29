// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ManualClimb extends Command {
  ClimbSubsystem m_climbSubsystem;
  BooleanSupplier m_leftUp, m_leftDown, m_rightUp, m_rightDown;

  /** Creates a new ManualClimb. */
  public ManualClimb(ClimbSubsystem subsystem, BooleanSupplier leftUp, BooleanSupplier leftDown, 
                     BooleanSupplier rightUp, BooleanSupplier rightDown) {
    m_climbSubsystem = subsystem;
    m_leftUp = leftUp;
    m_leftDown = leftDown;
    m_rightUp = rightUp;
    m_rightDown = rightDown;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbSubsystem.setManualMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climbSubsystem.manual(m_leftUp.getAsBoolean(), m_leftDown.getAsBoolean(), m_rightUp.getAsBoolean(), m_rightDown.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setManualMode(false);
    m_climbSubsystem.setMeasurementToZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

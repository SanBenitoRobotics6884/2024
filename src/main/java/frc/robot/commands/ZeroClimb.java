// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

import static frc.robot.Constants.Climb.VELOCITY_THRESHOLD;

public class ZeroClimb extends Command {
  ClimbSubsystem m_climbSubsystem;
  Timer m_timer = new Timer();
  LinearFilter m_leftVelocity = LinearFilter.movingAverage(10);
  LinearFilter m_rightVelocity = LinearFilter.movingAverage(10);

  /** Creates a new ZeroClimb. */
  public ZeroClimb(ClimbSubsystem subsystem) {
    m_climbSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    m_climbSubsystem.setZeroing(true);
    m_leftVelocity.reset();
    m_rightVelocity.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setZeroing(false);
    m_climbSubsystem.setMeasurementToRetractSetpoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(1.0) 
        && m_leftVelocity.calculate(m_climbSubsystem.getLeftVelocity()) < VELOCITY_THRESHOLD
        && m_rightVelocity.calculate(m_climbSubsystem.getRightVelocity()) < VELOCITY_THRESHOLD;
  }
}

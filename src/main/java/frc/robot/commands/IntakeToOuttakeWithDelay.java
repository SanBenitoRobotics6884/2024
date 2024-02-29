// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeToOuttakeWithDelay extends Command {
private IntakeSubsystem m_intakeSubsystem;
private Timer m_timer;
private double m_speed;
  /** Creates a new IntakesOutake. */
  public IntakeToOuttakeWithDelay(IntakeSubsystem subsystem, double speed) {
    m_intakeSubsystem = subsystem;
    m_speed = speed;
    m_timer = new Timer();
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(1.0)) {
      m_intakeSubsystem.roll(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.rollerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

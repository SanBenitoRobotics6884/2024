// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCommand extends Command {
  IntakeSubsystem m_intakesubsystem;
  /** Creates a new DeployCommand. */
  
  public DeployIntakeCommand(IntakeSubsystem subsystem) {
    m_intakesubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakesubsystem.deployGoal();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Reached goal");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakesubsystem.deployHitGoal();
  }
}

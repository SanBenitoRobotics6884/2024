// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;



public class ReelCommand extends Command {
  private IntakeSubsystem m_intakeSubsystem;
  
  /** Creates a new ReelCommand. */
  public ReelCommand(IntakeSubsystem subsystem) {


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // !!! ~ REMINDER: MAKE THE MOTOR INTAKE ~ !!! //
    m_intakeSubsystem.reel();
    System.out.println("Intaking/Reeling has begun");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_intakeSubsystem.rollerStop();
    System.out.println("Intaking/Reeling his hit a stop");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
return m_intakeSubsystem.noteHeld();
  }

   
}

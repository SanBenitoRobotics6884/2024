// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;



public class ReelCommand extends Command {
  private BooleanSupplier m_cancel;
  private IntakeSubsystem m_intakeSubsystem;
  
  /** Creates a new ReelCommand. */
  public ReelCommand(IntakeSubsystem subsystem, BooleanSupplier cancel) {
    m_intakeSubsystem = subsystem;
    m_cancel = cancel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intaking/Reeling has begun");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // !!! ~ REMINDER: MAKE THE MOTOR INTAKE ~ !!! //
    m_intakeSubsystem.reel();
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
    return m_cancel.getAsBoolean();
     
  }

   
}

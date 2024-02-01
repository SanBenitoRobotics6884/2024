// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;



public class ReelCommand extends Command {
  private IntakeSubsystem m_intakeSubsystem;
  private 
  
  /** Creates a new ReelCommand. */
  public ReelCommand(IntakeSubsystem subsystem, ) {


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
    double taken = IntakeSubsystem.m_intakeMotor.get();
    if (taken == 0) {
      IntakeSubsystem.m_intakeMotor.set(-5);
      //No idea what speed the motor is supposed to be set too
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

  }
}

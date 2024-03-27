// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.outtake.OuttakeSubsystem;

import static frc.robot.Constants.Intake.INTAKE_MOTOR_SPEAKER_SPEED;
import static frc.robot.Constants.Outtake.SPEAKER_PERCENT_OUTPUT;
import static frc.robot.Constants.ShootingData.*;

import java.util.function.Supplier;

public class DistanceShoot extends Command {
  private IntakeSubsystem m_intakeSubsystem;
  private OuttakeSubsystem m_outtakeSubsystem;
  private Supplier<Pose2d> m_pose;

  private Timer m_timer = new Timer();

  /** Creates a new DistanceShoot. */
  public DistanceShoot(IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem, Supplier<Pose2d> pose) {
    m_intakeSubsystem = intakeSubsystem;
    m_outtakeSubsystem = outtakeSubsystem;
    m_pose = pose;

    addRequirements(intakeSubsystem, outtakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double outtakeSetpoint = DISTANCE_TO_OUTTAKE_SETPOINT.get(Field.getDistanceMetersToSpeaker(m_pose.get()));
    double intakeSetpoint = OUTTAKE_TO_INTAKE_SETPOINT.get(outtakeSetpoint);
    m_outtakeSubsystem.setSetpoint(outtakeSetpoint);
    m_intakeSubsystem.setSetpoint(intakeSetpoint);

    m_outtakeSubsystem.rollOuttake(SPEAKER_PERCENT_OUTPUT);
    if (m_timer.hasElapsed(2.0)) {
      m_intakeSubsystem.roll(INTAKE_MOTOR_SPEAKER_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_outtakeSubsystem.stopMotors();
    m_intakeSubsystem.rollerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

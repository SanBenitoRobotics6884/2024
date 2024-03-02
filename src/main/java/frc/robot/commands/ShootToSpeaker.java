// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class ShootToSpeaker extends SequentialCommandGroup {
  IntakeSubsystem m_intakeSubsystem;
  OuttakeSubsystem m_outtakeSubsystem;

  /** Creates a new ShootToSpeaker. */
  public ShootToSpeaker(IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_outtakeSubsystem = outtakeSubsystem;

    addCommands(
        Commands.waitUntil(m_outtakeSubsystem::isNotZeroing),
        m_outtakeSubsystem.rotateToSpeakerCommand(),
        m_outtakeSubsystem.shootToSpeakerCommand()
        .alongWith(m_intakeSubsystem.getToSpeakerCommand())
        .withTimeout(3.0));
  }
}

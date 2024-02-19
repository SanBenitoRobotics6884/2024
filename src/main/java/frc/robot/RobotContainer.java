// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FieldDrive;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.commands.RotateOuttakeToAmp;
import frc.robot.commands.RotateOuttakeToSpeaker;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.Swerve.SQUARED_INPUTS;

public class RobotContainer {
  private CommandJoystick m_joystick = new CommandJoystick(0);

  private ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private CommandXboxController m_controller = new CommandXboxController(1);
  
  private SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();

  private DefaultDrive m_defaultDrive = new DefaultDrive(
      m_swerveSubsystem,
      () -> input(getLeftY(), SQUARED_INPUTS),
      () -> input(getLeftX(), SQUARED_INPUTS),
      () -> input(m_controller.getRightX(), SQUARED_INPUTS)); 

  private FieldDrive m_fieldDrive = new FieldDrive(
      m_swerveSubsystem,
      () -> input(getLeftY(), SQUARED_INPUTS),
      () -> input(getLeftX(), SQUARED_INPUTS),
      () -> input(m_controller.getRightX(), SQUARED_INPUTS)); 

  private RotateOuttakeToAmp m_ampPosition = new RotateOuttakeToAmp(m_outtakeSubsystem);
  private RotateOuttakeToSpeaker m_speakerPosition = new RotateOuttakeToSpeaker(m_outtakeSubsystem);

  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(m_defaultDrive);

    configureBindings();
  }

  private void configureBindings() {
    // Swerve bindings
    m_controller.a().toggleOnTrue(m_fieldDrive);
    m_controller.y().onTrue(Commands.runOnce(m_swerveSubsystem::zeroYaw));
    m_controller.b().onTrue(Commands.runOnce(m_swerveSubsystem::zeroPose));
    m_controller.x().onTrue(Commands.runOnce(m_swerveSubsystem::seedModuleMeasurements));
    
    // Climb bindings
    m_joystick.button(10).onTrue(m_climbSubsystem.getExtendCommand());
    m_joystick.button(9).onTrue(m_climbSubsystem.getRetractCommand());
    
    // Outtake bindings
    m_joystick.button(1).whileTrue(m_outtakeSubsystem.shootToSpeakerCommand());
    m_joystick.button(2).whileTrue(m_outtakeSubsystem.shootToAmpCommand());
    m_joystick.button(3).whileTrue(m_outtakeSubsystem.YoinkNoteCommand());

    m_joystick.button(4).onTrue(m_ampPosition);
    m_joystick.button(5).onTrue(m_speakerPosition);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private double input(double input, boolean squared) {
    return squared ? (input > 0 ? 1 : -1) * Math.pow(input, 2) : input;
  }

  private double getLeftY() {
    double leftY = m_controller.getLeftY();
    if (Math.hypot(m_controller.getLeftX(), leftY) < 0.1) {
      return 0;
    }
    return leftY;
  }

  private double getLeftX() {
    double leftX = m_controller.getLeftX();
    if (Math.hypot(leftX, m_controller.getLeftY()) < 0.1) {
      return 0;
    }
    return leftX;
  }
}

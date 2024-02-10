// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.FieldDrive;
import frc.robot.commands.IntakeToOuttake;
import frc.robot.commands.ReelCommand;
import frc.robot.commands.ReverseDeployCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.Swerve.SQUARED_INPUTS;

public class RobotContainer {
  private CommandXboxController m_controller = new CommandXboxController(1);
  private IntakeSubsystem m_intakesubsystem = new IntakeSubsystem();
  private SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private CommandJoystick m_joystick = new CommandJoystick(0);

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

  private ReelCommand m_reelcommand = new ReelCommand(m_intakesubsystem);
  private DeployIntakeCommand m_deployintakecommand = new DeployIntakeCommand(m_intakesubsystem);
  private IntakeToOuttake m_intaketoouttakecommand = new IntakeToOuttake(m_intakesubsystem);
  private ReverseDeployCommand m_reversedeploycommand = new ReverseDeployCommand(m_intakesubsystem);

  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(m_defaultDrive);
    configureBindings();

    
  }
  
  private void configureBindings() {
    m_joystick.button(0).onTrue(m_reelcommand);
    m_joystick.button(1).onTrue(m_deployintakecommand);
    m_joystick.button(2).onTrue(m_intaketoouttakecommand);
    m_joystick.button(3).onTrue(m_reversedeploycommand);
 
    m_controller.a().toggleOnTrue(m_fieldDrive);
    m_controller.y().onTrue(Commands.runOnce(m_swerveSubsystem::zeroYaw));
    m_controller.b().onTrue(Commands.runOnce(m_swerveSubsystem::zeroPose));
    m_controller.x().onTrue(Commands.runOnce(m_swerveSubsystem::seedModuleMeasurements));
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FieldDrive;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.Swerve.SQUARED_INPUTS;

public class RobotContainer {
  public enum BindingsSetting {
    PITS,
    CLIMB,
    COMPETITION;
  }
  private BindingsSetting setting = BindingsSetting.PITS;

  private CommandJoystick m_joystick = new CommandJoystick(0);
  private CommandXboxController m_controller;
  
  private SwerveSubsystem m_swerveSubsystem;
  private ClimbSubsystem m_climbSubsystem;
  private OuttakeSubsystem m_outtakeSubsystem ;
  private IntakeSubsystem m_intakeSubsystem;

  private DefaultDrive m_defaultDrive;

  private FieldDrive m_fieldDrive;

  public RobotContainer() {
    if (setting != BindingsSetting.CLIMB) {
      m_controller = new CommandXboxController(1);
      
      m_swerveSubsystem = new SwerveSubsystem();
      m_outtakeSubsystem = new OuttakeSubsystem();
      m_intakeSubsystem = new IntakeSubsystem();

      m_defaultDrive = new DefaultDrive(
          m_swerveSubsystem,
          () -> input(getLeftY(), SQUARED_INPUTS),
          () -> input(getLeftX(), SQUARED_INPUTS),
          () -> input(getRightX(), SQUARED_INPUTS)); 

      m_fieldDrive = new FieldDrive(
          m_swerveSubsystem,
          () -> input(getLeftY(), SQUARED_INPUTS),
          () -> input(getLeftX(), SQUARED_INPUTS),
          () -> input(getRightX(), SQUARED_INPUTS));

      m_swerveSubsystem.setDefaultCommand(m_fieldDrive);
    }
    m_climbSubsystem = new ClimbSubsystem();
    
    switch (setting) {
      case PITS:
        configureBindings();
        configurePitsBindings();
        break;
      case CLIMB:
        configureClimbBindings();
        break;
      default:
        configureBindings();
        configureCompetitionBindings();
        break;
    }
  }

  private void configureBindings() {
    // Swerve bindings
    // toggles between robot and field oriented
    m_controller.a().toggleOnTrue(m_defaultDrive);
    m_controller.y().onTrue(Commands.runOnce(m_swerveSubsystem::zeroYaw));
    // m_controller.x().onTrue(Commands.runOnce(m_swerveSubsystem::seedModuleMeasurements));
    
    // intake and outtake bindings
    // shoots to amp or speaker depending on position 
    m_joystick.trigger().whileTrue(Commands.either(
        m_outtakeSubsystem.shootToAmpCommand(), 
        m_outtakeSubsystem.shootToSpeakerCommand()
        .alongWith(m_intakeSubsystem.getToSpeakerCommand()), 
        m_outtakeSubsystem::isInAmpPosition));
    
    // Shoots (up into) amp
    m_joystick.top().whileTrue(Commands.parallel(
        m_outtakeSubsystem.shootToAmpCommand(),
        m_intakeSubsystem.getToAmpCommand()));
  }
  
  private void configureCompetitionBindings() {  
    // hands note piece from intake to outtake subsystem and rotates to amp
    m_joystick.button(3).onTrue(Commands.sequence(
        m_outtakeSubsystem.rotateToSpeakerCommand(),
        m_intakeSubsystem.getToAmpCommand()
            .alongWith(m_outtakeSubsystem.yoinkNoteCommand())
            .withTimeout(1.2),
        m_outtakeSubsystem.rotateToAmpPositionCommand()).
        withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    
    // Climb bindings
    // retracts climb
    m_joystick.button(10).onTrue(Commands.sequence(
        m_outtakeSubsystem.rotateToAmpPositionCommand()
        .alongWith(m_intakeSubsystem.getDeployCommand()),
        m_climbSubsystem.getRetractCommand())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // extends climb
    m_joystick.button(9).onTrue(Commands.sequence(
        m_outtakeSubsystem.rotateToAmpPositionCommand()
        .alongWith(m_intakeSubsystem.getDeployCommand()), 
        m_climbSubsystem.getExtendCommand())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // puts intake back in
    m_joystick.button(7).onTrue(m_intakeSubsystem.getStowCommand()
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // unnecessary?
    m_joystick.button(5).onTrue(m_outtakeSubsystem.rotateToAmpPositionCommand());
    m_joystick.button(6).onTrue(m_outtakeSubsystem.rotateToSpeakerCommand());
    
    // grabs the game piece
    m_joystick.button(11).onTrue(Commands.sequence(
        m_intakeSubsystem.getDeployCommand(),
        m_intakeSubsystem.getReelCommand(() -> m_joystick.getHID().getRawButton(12)),
        m_intakeSubsystem.getStowCommand())
        .alongWith(m_outtakeSubsystem.rotateToSpeakerCommand())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // ejects game piece
    m_joystick.button(12).onTrue(Commands.sequence(
        m_intakeSubsystem.getDeployCommand(),
        m_intakeSubsystem.getEjectCommand().withTimeout(2.0),
        m_intakeSubsystem.getStowCommand())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
  }

  private void configurePitsBindings() {
    // extend and retract
    m_joystick.button(10).onTrue(m_climbSubsystem.getExtendCommand());
    m_joystick.button(9).onTrue(m_climbSubsystem.getRetractCommand());

    // deploy and stow
    m_joystick.button(11).onTrue(m_intakeSubsystem.getDeployCommand());
    m_joystick.button(12).onTrue(m_intakeSubsystem.getStowCommand());

    m_joystick.button(8).whileTrue(m_intakeSubsystem.getSuckCommand());

    m_joystick.button(5).onTrue(m_outtakeSubsystem.rotateToAmpPositionCommand());
    m_joystick.button(6).onTrue(m_outtakeSubsystem.rotateToSpeakerCommand());
  }

  private void configureClimbBindings() {
    m_joystick.button(7).whileTrue(m_climbSubsystem.getManualCommand(
        () -> m_joystick.getHID().getRawButton(5),
        () -> m_joystick.getHID().getRawButton(3),
        () -> m_joystick.getHID().getRawButton(6),
        () -> m_joystick.getHID().getRawButton(4)));

    m_joystick.button(10).onTrue(m_climbSubsystem.getExtendCommand());
    m_joystick.button(9).onTrue(m_climbSubsystem.getRetractCommand());
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

  private double getRightX() {
    double rightX = m_controller.getRightX();
    if (Math.abs(rightX) < 0.1) {
      rightX = 0;
    }
    return rightX;
  }
  
}

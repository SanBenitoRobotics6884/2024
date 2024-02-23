// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.FieldDrive;
import frc.robot.commands.IntakeToOuttake;
import frc.robot.commands.ReelCommand;
import frc.robot.commands.StowIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.commands.RotateOuttakeToAmp;
import frc.robot.commands.RotateOuttakeToSpeaker;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.Swerve.SQUARED_INPUTS;

import java.util.function.BooleanSupplier;

public class RobotContainer {
  private CommandJoystick m_joystick = new CommandJoystick(0);
  // private CommandXboxController m_controller = new CommandXboxController(1);
  
  // private SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  // private ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();
  private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  // private DefaultDrive m_defaultDrive = new DefaultDrive(
      // m_swerveSubsystem,
      // () -> input(getLeftY(), SQUARED_INPUTS),
      // () -> input(getLeftX(), SQUARED_INPUTS),
      // () -> input(m_controller.getRightX(), SQUARED_INPUTS)); 

  // private FieldDrive m_fieldDrive = new FieldDrive(
      // m_swerveSubsystem,
      // () -> input(getLeftY(), SQUARED_INPUTS),
      // () -> input(getLeftX(), SQUARED_INPUTS),
      // () -> input(m_controller.getRightX(), SQUARED_INPUTS)); 

  private ReelCommand m_reelCommand = new ReelCommand(m_intakeSubsystem, () -> m_joystick.getHID().getRawButton(12));
  private DeployIntakeCommand m_deployIntakeCommand = new DeployIntakeCommand(m_intakeSubsystem);
  private IntakeToOuttake m_intakeToOuttakeCommand = new IntakeToOuttake(m_intakeSubsystem);
  private StowIntakeCommand m_stowCommand = new StowIntakeCommand(m_intakeSubsystem);

  // private RotateOuttakeToAmp m_ampPosition = new RotateOuttakeToAmp(m_outtakeSubsystem);
  // private RotateOuttakeToSpeaker m_speakerPosition = new RotateOuttakeToSpeaker(m_outtakeSubsystem);

  public RobotContainer() {
    // m_swerveSubsystem.setDefaultCommand(m_defaultDrive);
    
    configureBindings();
  }
  
  private void configureBindings() {
    /**
    // stows intake
    m_joystick.button(12).onTrue(m_intakeSubsystem.getStowCommand());

    // rotates to speaker
    m_joystick.button(4).onTrue(m_outtakeSubsystem.rotateToSpeakerCommand()
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // hands note piece from intake to outtake subsystem and rotates to amp
    m_joystick.button(3).onTrue(Commands.sequence(
        m_outtakeSubsystem.rotateToSpeakerCommand(),
        m_intakeSubsystem.getToOuttakeCommand()
            .alongWith(m_outtakeSubsystem.yoinkNoteCommand()
            .withTimeout(2.0)),
        m_outtakeSubsystem.rotateToAmpPositionCommand()).
        withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // shoots to amp or speaker depending on position
    */
    m_joystick.trigger().whileTrue(Commands.either(
        m_outtakeSubsystem.shootToAmpCommand(), 
        m_outtakeSubsystem.shootToSpeakerCommand()
        .alongWith(m_intakeSubsystem.getToOuttakeCommand()), 
        m_outtakeSubsystem::isInAmpPosition));
    /**

    // grabs the game piece
    m_joystick.button(11).onTrue(Commands.sequence(
        m_intakeSubsystem.getDeployCommand(),
        m_intakeSubsystem.getReelCommand(() -> m_joystick.getHID().getRawButton(12)),
        m_intakeSubsystem.getStowCommand())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    */
    // Climb bindings
    /**
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
    
    
    
    // Swerve bindings
    m_controller.a().toggleOnTrue(m_fieldDrive);
    m_controller.y().onTrue(Commands.runOnce(m_swerveSubsystem::zeroYaw));
    m_controller.x().onTrue(Commands.runOnce(m_swerveSubsystem::seedModuleMeasurements));
    
    // Climb bindings (testing)
    m_joystick.button(10).onTrue(m_climbSubsystem.getExtendCommand());
    m_joystick.button(9).onTrue(m_climbSubsystem.getRetractCommand());
    */
    // Outtake bindings (testing)
    // m_joystick.button(1).whileTrue(m_outtakeSubsystem.shootToSpeakerCommand());
    // m_joystick.button(2).whileTrue(m_outtakeSubsystem.shootToAmpCommand());
    // m_joystick.button(3).whileTrue(m_outtakeSubsystem.yoinkNoteCommand());
    
    m_joystick.button(5).onTrue(m_outtakeSubsystem.rotateToAmpPositionCommand());
    m_joystick.button(6).onTrue(m_outtakeSubsystem.rotateToSpeakerCommand());
    
    // Intake bindings (testing)
    m_joystick.button(4).whileTrue(m_reelCommand);
    m_joystick.button(11).onTrue(m_deployIntakeCommand);
    m_joystick.button(3).whileTrue(m_intakeToOuttakeCommand);
    m_joystick.button(12).onTrue(m_stowCommand);
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
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
  */
}

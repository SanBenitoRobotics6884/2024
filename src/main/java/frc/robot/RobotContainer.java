// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.commands.StowIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.commands.RotateOuttakeToAmp;
import frc.robot.commands.RotateOuttakeToSpeaker;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.Swerve.SQUARED_INPUTS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

public class RobotContainer {
  private CommandJoystick m_joystick = new CommandJoystick(0);
  private CommandXboxController m_controller = new CommandXboxController(1);
  
  private SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();
  private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

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

  private ReelCommand m_reelCommand = new ReelCommand(m_intakeSubsystem, () -> m_joystick.getHID().getRawButton(12));
  private DeployIntakeCommand m_deployIntakeCommand = new DeployIntakeCommand(m_intakeSubsystem);
  private IntakeToOuttake m_intakeToOuttakeCommand = new IntakeToOuttake(m_intakeSubsystem);
  private StowIntakeCommand m_stowCommand = new StowIntakeCommand(m_intakeSubsystem);

  private RotateOuttakeToAmp m_ampPosition = new RotateOuttakeToAmp(m_outtakeSubsystem);
  private RotateOuttakeToSpeaker m_speakerPosition = new RotateOuttakeToSpeaker(m_outtakeSubsystem);

  public RobotContainer() {
    NamedCommands.registerCommand("Intake", m_deployIntakeCommand);
    NamedCommands.registerCommand("Scoring", m_outtakeSubsystem.shootToSpeakerCommand());
    m_swerveSubsystem.setDefaultCommand(m_defaultDrive);
    
    configureBindings();
  }
  
  private void configureBindings() {
    // Swerve bindings
    m_controller.a().toggleOnTrue(m_fieldDrive);
    m_controller.y().onTrue(Commands.runOnce(m_swerveSubsystem::zeroYaw));
    m_controller.b().onTrue(Commands.runOnce(m_swerveSubsystem::zeroPose));
    m_controller.x().onTrue(Commands.runOnce(m_swerveSubsystem::seedModuleMeasurements));
    
    /**
    // Climb bindings
    m_joystick.button(10).onTrue(m_climbSubsystem.getExtendCommand());
    m_joystick.button(9).onTrue(m_climbSubsystem.getRetractCommand());
    
    // Outtake bindings
    m_joystick.button(1).whileTrue(m_outtakeSubsystem.shootToSpeakerCommand());
    m_joystick.button(2).whileTrue(m_outtakeSubsystem.shootToAmpCommand());
    m_joystick.button(3).whileTrue(m_outtakeSubsystem.YoinkNoteCommand());

    m_joystick.button(4).onTrue(m_ampPosition);
    m_joystick.button(5).onTrue(m_speakerPosition);
    
    // Intake bindings
    m_joystick.button(0).onTrue(m_reelcommand);
    m_joystick.button(1).onTrue(m_deployintakecommand);
    m_joystick.button(2).onTrue(m_intaketoouttakecommand);
    m_joystick.button(3).onTrue(m_stowcommand);
    */
    
// Since we are using a holonomic drivetrain, the rotation component of this pose
// represents the goal holonomic rotation
Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));

// Create the constraints to use while pathfinding
PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        
        
);

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

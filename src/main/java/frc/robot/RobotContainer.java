// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Swerve.SQUARED_INPUTS;
import static frc.robot.Constants.Vision.SHOOTER_CAMERA;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.outtake.OuttakeIOReal;
import frc.robot.subsystems.outtake.OuttakeSubsystem;
import frc.robot.Constants.Field;
import frc.robot.commands.DistanceShoot;
import frc.robot.commands.ShootToSpeaker;
import frc.robot.commands.TuneDistanceShoot;
import frc.robot.subsystems.climb.ClimbIOSparkMax;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.RobotDrive;
import frc.robot.subsystems.swerve.FieldDrive;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  public enum BindingsSetting {
    PITS,
    CLIMB,
    TUNE_SHOOTER,
    COMPETITION;
  }

  private BindingsSetting m_setting = BindingsSetting.COMPETITION;

  private CommandJoystick m_joystick;
  private CommandXboxController m_controller;
  
  private SwerveSubsystem m_swerveSubsystem;
  private ClimbSubsystem m_climbSubsystem;
  private OuttakeSubsystem m_outtakeSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private VisionSubsystem m_visionSubsystem;

  private RobotDrive m_defaultDrive;
  private FieldDrive m_slowDrive;
  private FieldDrive m_fieldDrive;

  private LoggedDashboardChooser<AutoCommand> m_autoSelector = new LoggedDashboardChooser<>("chooser");

  public RobotContainer() {
    m_joystick = new CommandJoystick(0);
    
    m_climbSubsystem = new ClimbSubsystem(new ClimbIOSparkMax());
    if (m_setting == BindingsSetting.CLIMB) {
      configureClimbBindings();
      return;
    }
    m_controller = new CommandXboxController(1);
    
    m_swerveSubsystem = new SwerveSubsystem();
    m_outtakeSubsystem = new OuttakeSubsystem(new OuttakeIOReal());
    m_intakeSubsystem = new IntakeSubsystem(new IntakeIOReal());
    m_visionSubsystem = new VisionSubsystem(m_swerveSubsystem, new VisionIOPhoton(SHOOTER_CAMERA));
    
    m_slowDrive = new FieldDrive(
        m_swerveSubsystem,
        () -> 0.5 * getLeftY(),
        () -> 0.5 * getLeftX(),
        () -> 0.5 * getRightX());

    m_defaultDrive = new RobotDrive(
        m_swerveSubsystem,
        () -> 3 * getLeftY(),
        () -> 3 * getLeftX(),
        () -> 3 * getRightX()); 

    m_fieldDrive = new FieldDrive(
        m_swerveSubsystem,
        () -> 3 * getLeftY(),
        () -> 3 * getLeftX(),
        () -> 3 * getRightX());

    m_swerveSubsystem.setDefaultCommand(m_fieldDrive);

    switch (m_setting) {
      case PITS:
        configureBindings();
        configurePitsBindings();
        break;
      case TUNE_SHOOTER:
        configureBindings();
        configureIntakeBindings();
        configureTuneShootingBindings();
      default:
        configureBindings();
        configureCompetitionBindings();
        configureIntakeBindings();
        break;
    }

    PathPlannerLogging.setLogTargetPoseCallback(pose -> 
        Logger.recordOutput("pathplanner/target-pose", pose));
    PathPlannerLogging.setLogActivePathCallback(poses -> 
        Logger.recordOutput("pathplanner/target-poses", poses.toArray(new Pose2d[0])));

    NamedCommands.registerCommand("intake", Commands.sequence(
        m_intakeSubsystem.getDeployCommand(), 
        m_intakeSubsystem.getReelCommand(() -> false).withTimeout(3.0), 
        m_intakeSubsystem.getStowCommand()));
    NamedCommands.registerCommand("score", 
        new ShootToSpeaker(m_intakeSubsystem, m_outtakeSubsystem));

    m_autoSelector.addDefaultOption("none", new AutoCommand(Commands.none(), 0));
    // We have not tested any of these
    addPPAuto("amp-side shoot and pick-up", "Left Pick Up");
    addPPAuto("amp-side taxi", "Left Taxi");
    addPPAuto("center shoot and pick-up", "Center Pick Up");
    addPPAuto("source-side shoot and pick-up", "Right Pick Up");
    addPPAuto("source-side taxi", "Right Taxi");
    addPPAuto("tune", "Tune");
    addSimpleAuto("left shoot", 
        new ShootToSpeaker(m_intakeSubsystem, m_outtakeSubsystem), 60.0);
    addSimpleAuto("center shoot", 
        new ShootToSpeaker(m_intakeSubsystem, m_outtakeSubsystem), 0);
    addSimpleAuto("right shoot", 
        new ShootToSpeaker(m_intakeSubsystem, m_outtakeSubsystem), -60.0);
  }

  /** These bindings are in both the COMPETITION and PITS binding modes */
  private void configureBindings() {
    // Swerve bindings
    // drives slow while left bumper pressed
    
    m_controller.leftBumper().whileTrue(m_slowDrive);

    // toggles between robot and field oriented
    m_controller.a().toggleOnTrue(m_defaultDrive);

    // sets the forward facing angle for field oriented drive
    m_controller.y().onTrue(Commands.runOnce(m_swerveSubsystem::zeroYaw));
    // m_controller.x().onTrue(Commands.runOnce(m_swerveSubsystem::seedModuleMeasurements));

    m_controller.povDown().whileTrue(pathfindToPose(Field.SPEAKER_SHOOTING_POSE));
    m_controller.povLeft().whileTrue(pathfindToPose(Field.AMP_SHOOTING_POSE));
    m_controller.povRight().whileTrue(pathfindToPose(Field.SOURCE_GETTING_POSE));

    m_joystick.povUp().onTrue(Commands.runOnce(m_visionSubsystem::takeSnapshot));
  }
  
  /** These are bindings in the COMPETITION bindings mode */
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

    // unnecessary?
    m_joystick.button(5).onTrue(m_outtakeSubsystem.rotateToAmpPositionCommand());
    m_joystick.button(6).onTrue(m_outtakeSubsystem.rotateToSpeakerCommand());
    
    // shoots to amp or speaker depending on position 
    m_joystick.trigger().whileTrue(Commands.either(
        m_outtakeSubsystem.shootToAmpCommand(), 
        m_outtakeSubsystem.shootToSpeakerCommand()
        .alongWith(m_intakeSubsystem.getToSpeakerCommand()), 
        m_outtakeSubsystem::isInAmpPosition));

    m_joystick.top().whileTrue(new DistanceShoot(
        m_intakeSubsystem, m_outtakeSubsystem, m_swerveSubsystem::getPose));
  }

  private void configureIntakeBindings() {
    // puts intake back in
    m_joystick.button(7).onTrue(m_intakeSubsystem.getStowCommand()
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

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
        m_intakeSubsystem.getEjectCommand().withTimeout(1.5))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

  }

  /** These are bindings in the PITS bindings mode */
  private void configurePitsBindings() {
    // extend and retract
    m_joystick.button(9).onTrue(m_climbSubsystem.getExtendCommand());
    m_joystick.button(10).onTrue(m_climbSubsystem.getRetractCommand());

    // deploy and stow
    m_joystick.button(11).onTrue(m_intakeSubsystem.getDeployCommand());
    m_joystick.button(12).onTrue(m_intakeSubsystem.getStowCommand());
    
    // Pick up and release note
    m_joystick.button(8).whileTrue(m_intakeSubsystem.getSuckCommand());
    m_joystick.button(7).whileTrue(m_intakeSubsystem.getEjectCommand());

    // rotate outtake to amp or speaker position
    m_joystick.button(5).onTrue(m_outtakeSubsystem.rotateToAmpPositionCommand());
    m_joystick.button(6).onTrue(m_outtakeSubsystem.rotateToSpeakerCommand());

    m_joystick.trigger().whileTrue(m_outtakeSubsystem.shootToSpeakerCommand());

    m_joystick.top().whileTrue(m_intakeSubsystem.getEjectCommand());
  }

  /** These are the only bindings when in the CLIMB calibration bindings mode */
  private void configureClimbBindings() {
    // calibrate climb manually
    m_joystick.button(7).whileTrue(m_climbSubsystem.getManualCommand(
        () -> m_joystick.getHID().getRawButton(5),
        () -> m_joystick.getHID().getRawButton(3),
        () -> m_joystick.getHID().getRawButton(6),
        () -> m_joystick.getHID().getRawButton(4)));

    // extends and retracts climb
    m_joystick.button(9).onTrue(m_climbSubsystem.getExtendCommand());
    m_joystick.button(10).onTrue(m_climbSubsystem.getRetractCommand());
  }

  /** These are bindings in the TUNE_SHOOTING mode */
  private void configureTuneShootingBindings() {
    m_joystick.button(7).whileTrue(new TuneDistanceShoot(
        m_intakeSubsystem, m_outtakeSubsystem, m_swerveSubsystem::getPose, 
        () -> m_joystick.getHID().getRawButtonPressed(5),
        () -> m_joystick.getHID().getRawButtonPressed(3),
        () -> m_joystick.getHID().getRawButtonPressed(6),
        () -> m_joystick.getHID().getRawButtonPressed(4),
        () -> m_joystick.getHID().getTopPressed(),
        () -> m_joystick.getHID().getTrigger()));
  }

  private Command pathfindToPose(Pose2d pose) {
    return AutoBuilder.pathfindToPose(pose, new PathConstraints(1.0, 1.0, Math.PI, Math.PI));
  }

  public Command getAutonomousCommand() {
    AutoCommand auto = m_autoSelector.get();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      auto.flip();
    }
    return Commands.runOnce(() -> m_swerveSubsystem.setYaw(auto.getAngle())).andThen(auto.getCommand());
  }

  private void addPPAuto(String name, String autoName) {
    m_autoSelector.addOption(name, AutoCommand.fromPPAuto(autoName));
  }

  private void addSimpleAuto(String name, Command command, double angleDegrees) {
    m_autoSelector.addOption(name, new AutoCommand(command, angleDegrees));
  }

  private double getLeftY() {
    double leftX = m_controller.getLeftX();
    double leftY = m_controller.getLeftY();
    if (Math.hypot(leftX, leftY) < 0.1) {
      return 0;
    }
    if (!SQUARED_INPUTS) {
      return leftY;
    }
    double angle = Math.atan2(leftY, leftX);
    double magnitude = MathUtil.clamp(Math.pow(Math.hypot(leftX, leftY), 2), -1, 1);
    return magnitude * Math.sin(angle);
  }

  private double getLeftX() {
    double leftX = m_controller.getLeftX();
    double leftY = m_controller.getLeftY();
    if (Math.hypot(leftX, leftY) < 0.1) {
      return 0;
    }
    if (!SQUARED_INPUTS) {
      return leftX;
    }
    double angle = Math.atan2(leftY, leftX);
    double magnitude = MathUtil.clamp(Math.pow(Math.hypot(leftX, leftY), 2), -1, 1);
    return magnitude * Math.cos(angle);
  }

  private double getRightX() {
    double rightX = m_controller.getRightX();
    if (Math.abs(rightX) < 0.1) {
      rightX = 0;
    }
    return rightX;
  }

  private static class AutoCommand {
    private Command m_command;
    private double m_angleDegrees;

    AutoCommand(Command command, double angleDegrees) {
      m_command = command;
      m_angleDegrees = angleDegrees;
    }

    Command getCommand() {
      return m_command;
    }

    double getAngle() {
      return m_angleDegrees;
    }

    void flip() {
      m_angleDegrees = -m_angleDegrees;
    }


    static AutoCommand fromPPAuto(String name) {
      return new AutoCommand(
          new PathPlannerAuto(name), 
          PathPlannerAuto.getStaringPoseFromAutoFile(name).getRotation().getDegrees());
    }

  }
  
}

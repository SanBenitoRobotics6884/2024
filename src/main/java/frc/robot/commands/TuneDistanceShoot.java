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
import static frc.robot.Constants.Intake.STOW_SETPOINT;
import static frc.robot.Constants.Outtake.SPEAKER_PERCENT_OUTPUT;
import static frc.robot.Constants.Outtake.SPEAKER_POSITION;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class TuneDistanceShoot extends Command {
  private IntakeSubsystem m_intakeSubsystem;
  private OuttakeSubsystem m_outtakeSubsystem;
  private Supplier<Pose2d> m_pose;

  private double m_intakeSetpoint = STOW_SETPOINT;
  private double m_outtakeSetpoint = SPEAKER_POSITION;

  private Timer m_timer = new Timer();

  private BooleanSupplier m_increaseIntake;
  private BooleanSupplier m_decreaseIntake;
  private BooleanSupplier m_increaseOuttake;
  private BooleanSupplier m_decreaseOuttake;
  private BooleanSupplier m_printSetpoints;
  private BooleanSupplier m_shoot;

  private boolean m_startedShooting = false;

  /** Creates a new DistanceShoot. */
  public TuneDistanceShoot(IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem, Supplier<Pose2d> pose, 
                           BooleanSupplier increaseIntake, BooleanSupplier decreaseIntake, 
                           BooleanSupplier increaseOuttake, BooleanSupplier decreaseOuttake, 
                           BooleanSupplier printSetpoints, BooleanSupplier shoot) {
    m_intakeSubsystem = intakeSubsystem;
    m_outtakeSubsystem = outtakeSubsystem;
    m_pose = pose;
    m_increaseIntake = increaseIntake;
    m_decreaseIntake = decreaseIntake;
    m_increaseOuttake = increaseOuttake;
    m_decreaseOuttake = decreaseOuttake;
    m_printSetpoints = printSetpoints;
    m_shoot = shoot;

    addRequirements(intakeSubsystem, outtakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startedShooting = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean increaseIntake = m_increaseIntake.getAsBoolean();
    boolean decreaseIntake = m_decreaseIntake.getAsBoolean();
    if (increaseIntake ^ decreaseIntake) {
      m_intakeSetpoint += 0.1 * (increaseIntake ? -1 : 1);
    }

    boolean increaseOuttake = m_increaseOuttake.getAsBoolean();
    boolean decreaseOuttake = m_decreaseOuttake.getAsBoolean();
    if (increaseOuttake ^ decreaseOuttake) {
      m_outtakeSetpoint += 0.02 * (increaseOuttake ? -1 : 1);
    }

    if (m_printSetpoints.getAsBoolean()) {
      System.out.println("distance: " + Field.getDistanceMetersToSpeaker(m_pose.get()) 
                         + " outtake: " + m_outtakeSetpoint 
                         + " intake: " + m_intakeSetpoint);
    }

    m_outtakeSubsystem.setSetpoint(m_outtakeSetpoint);
    m_intakeSubsystem.setSetpoint(m_intakeSetpoint);

    if (m_shoot.getAsBoolean() && !m_startedShooting) {
      m_startedShooting = true;
      m_timer.restart();
    }

    if (!m_shoot.getAsBoolean() && m_startedShooting) {
      m_startedShooting = false;
    }

    if (m_startedShooting) {
      m_outtakeSubsystem.rollOuttake(SPEAKER_PERCENT_OUTPUT);
      if (m_timer.hasElapsed(2.0)) {
        m_intakeSubsystem.roll(INTAKE_MOTOR_SPEAKER_SPEED);
      }
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
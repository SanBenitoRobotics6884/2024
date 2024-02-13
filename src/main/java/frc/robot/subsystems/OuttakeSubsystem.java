// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Outtake.*;

public class OuttakeSubsystem extends SubsystemBase {
  CANSparkMax m_takeNoteMotor = new CANSparkMax(TAKE_NOTE_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax m_shooterMotorI = new CANSparkMax(SHOOTER_MOTOR_I_ID, MotorType.kBrushless);
  CANSparkMax m_shooterMotorII = new CANSparkMax(SHOOTER_MOTOR_II_ID, MotorType.kBrushless);
  CANSparkMax m_pivotMotor = new CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

  PIDController m_PID = new PIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD);

  RelativeEncoder m_pivotEncoder = m_pivotMotor.getEncoder();

  double m_pivotSetpoint = 0;

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    m_shooterMotorI.follow(m_shooterMotorII);
    m_shooterMotorI.setInverted(SHOOTER_MOTOR_I_INVERTED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pivotMotor.set(m_PID.calculate(m_pivotEncoder.getPosition(), m_pivotSetpoint));

    if (atSetpoint() && m_pivotSetpoint == SPEAKER_POSITION) {
      m_pivotMotor.setVoltage(0);
    }
  }

  public void toSpeakerPosition() {
    m_pivotSetpoint = SPEAKER_POSITION;
  }

  public void toAmpPosition() {
    m_pivotSetpoint = AMP_POSITION;
  }

  public boolean atSetpoint() {
    return Math.abs(m_pivotEncoder.getPosition() - m_pivotSetpoint) < TOLERANCE;
  }

  public void rollOuttake(double takeNoteSpeed, double shootersSpeed) {
    m_takeNoteMotor.set(takeNoteSpeed);
    m_shooterMotorII.set(shootersSpeed);
  }

  public Command shootToAmpCommand() {
    return run(() -> rollOuttake(TAKE_NOTE_AMP_MOTOR_VOLTAGE, SHOOTER_AMP_MOTOR_VOLTAGE));
  }

  public Command shootToSpeakerCommand() {
    return run(() -> rollOuttake(TAKE_NOTE_SPEAKER_MOTOR_VOLTAGE, SHOOTER_SPEAKER_MOTOR_VOLTAGE));
  }

  public Command YoinkNoteCommand() {
    return run(() -> rollOuttake(YOINK_TAKE_NOTE_SPEED, YOINK_SHOOTERS_SPEED));
  }

  public Command rotateToAmpPositionCommand() {
    return new FunctionalCommand(
        this::toAmpPosition, 
        () -> {}, 
        b -> {},
        this::atSetpoint, 
        this);
  }

  public Command rotateToSpeakerCommand() {
    return new FunctionalCommand(
        this::toSpeakerPosition,
        () -> {},
        a -> {},
        this::atSetpoint,
        this);
  }
}

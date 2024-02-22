// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  DigitalInput m_ampLimitSwitch = new DigitalInput(AMP_LIMIT_SWITCH_CHANNEL);

  double m_pivotSetpoint = 0;
  boolean isZeroing = false; // Should initially be true

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    m_shooterMotorI.follow(m_shooterMotorII, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isZeroing) {
      m_pivotMotor.setVoltage(ZEROING_VOLTAGE);
      if (m_ampLimitSwitch.get()) {
        isZeroing = false;
        m_pivotEncoder.setPosition(0);
      }
    } else {
      m_pivotMotor.set(m_PID.calculate(m_pivotEncoder.getPosition(), m_pivotSetpoint));
    }
  }

  public boolean isAtAmpPosition() {
    return m_ampLimitSwitch.get();
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

  public void stopMotors() {
    m_takeNoteMotor.stopMotor();
    m_shooterMotorI.stopMotor();
    m_shooterMotorII.stopMotor();
  }

  // The following code it is not used, but we love it, so we're leaving it here. :fire: :skull:

  public Command shootToAmpCommand() {
    return run(() -> rollOuttake(TAKE_NOTE_AMP_MOTOR_VOLTAGE, SHOOTER_AMP_MOTOR_VOLTAGE)).finallyDo(this::stopMotors);
  }

  public Command shootToSpeakerCommand() {
    return run(() -> rollOuttake(TAKE_NOTE_SPEAKER_MOTOR_VOLTAGE, SHOOTER_SPEAKER_MOTOR_VOLTAGE)).finallyDo(this::stopMotors);
  }

  public Command YoinkNoteCommand() {
    return run(() -> rollOuttake(YOINK_TAKE_NOTE_SPEED, YOINK_SHOOTERS_SPEED)).finallyDo(this::stopMotors);
  }

  // The two following commands make the robot outtake to rotate either amp or speaker possition. :fire: :sob:

  public Command rotateToAmpPositionCommand() {
    return Commands.runOnce(this::toAmpPosition);
  }

  public Command rotateToSpeakerCommand() {
    return Commands.runOnce(this::toSpeakerPosition);
  }
}

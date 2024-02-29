// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  DigitalInput m_ampLimitSwitch = new DigitalInput(AMP_LIMIT_SWITCH_CHANNEL);

  double m_pivotSetpoint = 0;
  boolean isZeroing = true; // Should initially be true
  double maxLeftCurrent = 0;
  double maxRightCurrent = 0;
  double maxPassOffCurrent = 0;

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    m_shooterMotorI.follow(m_shooterMotorII, true);
    m_shooterMotorI.setIdleMode(IdleMode.kBrake);
    m_shooterMotorII.setIdleMode(IdleMode.kBrake);

    m_shooterMotorI.setSmartCurrentLimit(90);
    m_shooterMotorII.setSmartCurrentLimit(90);
    m_takeNoteMotor.setSmartCurrentLimit(90);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isZeroing) {
      m_pivotMotor.set(ZEROING_VOLTAGE);
      if (m_ampLimitSwitch.get()) {
        isZeroing = false;
        m_pivotEncoder.setPosition(0);
      }
    } else {
      m_pivotMotor.set(m_PID.calculate(m_pivotEncoder.getPosition(), m_pivotSetpoint));
    }

    SmartDashboard.putBoolean("switch", m_ampLimitSwitch.get());
    // SmartDashboard.putBoolean("zeroing", isZeroing);
    // SmartDashboard.putNumber("left shooter", maxLeftCurrent);
    // SmartDashboard.putNumber("right shooter", maxRightCurrent);
    // SmartDashboard.putNumber("pass off motor", maxPassOffCurrent);

    double leftCurrent = m_shooterMotorI.getOutputCurrent();
    double rightCurrent = m_shooterMotorII.getOutputCurrent();
    double passOffCurrent = m_takeNoteMotor.getOutputCurrent();
    if (maxLeftCurrent < leftCurrent) {
      maxLeftCurrent = leftCurrent;
    }
    if (maxRightCurrent < rightCurrent) {
      maxRightCurrent = rightCurrent;
    }
    if (maxPassOffCurrent < passOffCurrent) {
      maxPassOffCurrent = passOffCurrent;
    }
  }

  public boolean ampLimitSwitchHit() {
    return m_ampLimitSwitch.get();
  }

  public boolean isInAmpPosition() {
    return m_pivotEncoder.getPosition() > (SPEAKER_POSITION + AMP_POSITION) / 2.0;
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

  public Command yoinkNoteCommand() {
    return run(() -> rollOuttake(YOINK_TAKE_NOTE_SPEED, YOINK_SHOOTERS_SPEED)).finallyDo(this::stopMotors);
  }

  // The two following commands make the robot outtake to rotate either amp or speaker possition. :fire: :sob:

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
      b -> {}, 
      this::atSetpoint, 
      this);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Outtake.*;

public class OuttakeSubsystem extends SubsystemBase {
  XboxController m_controller = new XboxController(0);

  CANSparkMax m_takeNoteMotor = new CANSparkMax(TAKE_NOTE_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax m_shooterMotorI = new CANSparkMax(SHOOTER_MOTOR_I_ID, MotorType.kBrushless);
  CANSparkMax m_shooterMotorII = new CANSparkMax(SHOOTER_MOTOR_II_ID, MotorType.kBrushless);
  CANSparkMax m_pivotMotor = new CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

  PIDController m_PID = new PIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD);

  RelativeEncoder m_pivotEncoder = m_pivotMotor.getEncoder();

  double m_pivotSetPoint = 0;

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem(XboxController controller) {
    m_controller = controller;
    m_shooterMotorI.follow(m_shooterMotorII);
    m_shooterMotorII.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pivotMotor.set(m_PID.calculate(m_pivotEncoder.getPosition(), m_pivotSetPoint));
  }

  public void ShootToSpeaker() {
    m_takeNoteMotor.setVoltage(TAKE_NOTE_MOTOR_VOLTAGE);


  }

  public void toSpeakerPosition() {
    m_pivotSetPoint = SPEAKER_POSITION;
    
  }

  public void toAmpPosition() {
    m_pivotSetPoint = AMP_POSITION;
  }
}

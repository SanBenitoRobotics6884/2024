// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
  XboxController m_controller = new XboxController(0);

  CANSparkMax m_takeNoteMotor = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax m_shooterMotorI = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_shooterMotorII = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_pivotMotor = new CANSparkMax(3, MotorType.kBrushless);
  
  PIDController m_pid = new PIDController(0, 0, 0);

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem(XboxController controller) {
    m_controller = controller;
    m_shooterMotorI.follow(m_shooterMotorII);
    m_shooterMotorII.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}

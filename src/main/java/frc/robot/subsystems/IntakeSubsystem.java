// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;


public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_pivotMotor = new CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
  
    //double kP = 0;
    //double kI = 0;    SET THE VARIABLES TO THE CONSTANTS. :fire: :skull:
    //double kD = 0;

  private ProfiledPIDController m_pivotPID = new ProfiledPIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD, null);
  private  DigitalInput m_limitSwitch = new DigitalInput(0);
 
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    RelativeEncoder m_intakeEncoder;
    RelativeEncoder m_pivotEncoder;

    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //m_intakeMotor.setSmartCurrentLimit(0); UNNEEDED ATM
    m_intakeEncoder = m_intakeMotor.getEncoder();
    

    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //m_pivotMotor.setSmartCurrentLimit(0); UNNEEDED ATM
    m_pivotEncoder = m_pivotMotor.getEncoder();

    /**
      Saw this in cranberry code, it sets like current limit in AMPs though, I don't know what it is so LUCAS help.
    */
    m_intakeMotor.set(INTAKE_MOTOR_SPEED);
    m_pivotMotor.set(PIVOT_MOTOR_SPEED);

    m_pivotMotor.getPIDController();
  }

 @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }
//This method will hopefully be used within the deploy command, still WIP atm.
//It 
  public void pid() {

  }
  
  public void reelBegin() {
    m_intakeMotor.set(1);
  }

  public boolean noteHeld() {
    return m_limitSwitch.get();
  }

  public void reelStop() {
    if (noteHeld() == true) {
      m_intakeMotor.stopMotor();
    }
  }

  
}
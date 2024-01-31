// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {

  final CANSparkMax m_intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
  final  CANSparkMax m_pivotMotor = new CANSparkMax(0, MotorType.kBrushless);
  
    double kP = 0;
    double kI = 0;
    double kD = 0;
  
  final AnalogEncoder m_pivotEncoder = new AnalogEncoder(0);
  final AnalogEncoder m_motorEncoder = new AnalogEncoder(0);
  final ProfiledPIDController m_pivotPID = new ProfiledPIDController(kP, kI, kD, null);
  final DigitalInput m_limitSwitch = new DigitalInput(0);
 
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(null);

    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_pivotMotor.setSmartCurrentLimit(0);
    /**
      Saw this in cranberry code, it sets like current limit in AMPs though, I don't know what it is so LUCAS help.
    */
    m_intakeMotor.set(0);
    m_pivotMotor.set(0);

    
  }

 @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    
    if (m_limitSwitch.get() == true) {
      
    }

    if (m_pivotEncoder.get() == 0) {
      //m_pivotMotor
    }
  }
}

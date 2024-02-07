// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
  /** Creates a new ClimbSubsystem. */
    private CANSparkMax m_rightClimbMotor = new CANSparkMax(0,MotorType.kBrushless);
    private CANSparkMax m_leftClimbMotor = new CANSparkMax(0,MotorType.kBrushless);
    
    private PIDController m_rightPIDController = new PIDController(0, 0, 0);
    private PIDController m_leftPIDController = new PIDController(0, 0, 0);

    private RelativeEncoder m_rightClimbEncoder;
    private RelativeEncoder m_leftClimbEncoder;

    private double m_rightSetPoint;
    private double m_leftSetPoint;

    private boolean isExtended = false;
     

    public ClimbSubsystem () {
    m_rightClimbEncoder = m_rightClimbMotor.getEncoder();
    m_leftClimbEncoder = m_leftClimbMotor.getEncoder();
    m_rightClimbEncoder.setPosition(0);
    m_leftClimbEncoder.setPosition(0);
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_rightPIDController.setSetpoint(0);
    m_leftPIDController.setSetpoint(0);

    double m_rightCurrentHeight = m_rightClimbEncoder.getPosition();
    double m_leftCurrentHeight = m_leftClimbEncoder.getPosition();

    double m_rightMotorOutput = m_rightPIDController.calculate(m_rightCurrentHeight);
    double m_leftMotorOutput = m_leftPIDController.calculate(m_leftCurrentHeight);

    m_rightMotorOutput = Math.min(0, Math.max(0, 0));
    m_leftMotorOutput = Math.min(0, Math.max(0, 0));
    
  }

public void toggleClimb() {
  if(isExtended) 
    extend();
  else 
    retract();
  }

  public void extend(){

    double m_rightMotorOutput = m_rightPIDController.calculate(0);
    double m_leftMotorOutput= m_leftPIDController.calculate(0);

    m_rightClimbMotor.set(m_rightMotorOutput);
    m_leftClimbMotor.set(m_leftMotorOutput);
    isExtended = true;
  }

  public void retract() {   
    double m_rightMotorOutput = m_rightPIDController.calculate(0);
    double m_leftMotorOutput = m_leftPIDController.calculate(0);
   
   
    m_rightClimbMotor.set(m_rightMotorOutput);
    m_leftClimbMotor.set(m_leftMotorOutput);
    isExtended = false;
  }
  public void stopClimb(){
    m_rightClimbMotor.set(0);
    m_leftClimbMotor.set(0);
   }
}
   
  

  
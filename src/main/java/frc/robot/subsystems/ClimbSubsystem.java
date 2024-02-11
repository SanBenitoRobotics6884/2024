// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climb.*;

public class ClimbSubsystem extends SubsystemBase{
 
  
  
  /** Creates a new ClimbSubsystem. */
    private CANSparkMax m_rightClimbMotor = new CANSparkMax(RCLIMB_MOTOR_ID,MotorType.kBrushless);
    private CANSparkMax m_leftClimbMotor = new CANSparkMax(LCLIMB_MOTOR_ID,MotorType.kBrushless);
    
    private PIDController m_rightPIDController = new PIDController(CLIMB_kP, CLIMB_kI, CLIMB_kD);
    private PIDController m_leftPIDController = new PIDController(CLIMB_kP, CLIMB_kI, CLIMB_kD);
        
    private RelativeEncoder m_rightClimbEncoder;
    private RelativeEncoder m_leftClimbEncoder;

    public ClimbSubsystem () {
    m_rightClimbEncoder = m_rightClimbMotor.getEncoder();
    m_leftClimbEncoder = m_leftClimbMotor.getEncoder();
    m_rightClimbEncoder.setPosition(0);
    m_leftClimbEncoder.setPosition(0);
    m_rightClimbMotor.setIdleMode(IdleMode.kBrake);
    m_leftClimbMotor.setIdleMode(IdleMode.kBrake);
  }
   @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double m_rightCurrentHeight = m_rightClimbEncoder.getPosition();
    double m_leftCurrentHeight = m_leftClimbEncoder.getPosition();
    
    double m_rightMotorOutput = m_rightPIDController.calculate(m_rightCurrentHeight);
    double m_leftMotorOutput = m_leftPIDController.calculate(m_leftCurrentHeight);
   
  
    m_rightClimbMotor.set(MathUtil.clamp(m_rightMotorOutput, MAX_DOWN_VOLTAGE,MAX_UP_VOLTAGE )); 
    m_leftClimbMotor.set(MathUtil.clamp(m_leftMotorOutput, MAX_DOWN_VOLTAGE, MAX_UP_VOLTAGE)); 
    m_leftClimbEncoder.setPosition(m_leftMotorOutput); 
    m_rightClimbEncoder.setPosition(m_rightMotorOutput); 

}
  
  public void extend(){
    m_rightPIDController.setSetpoint(EXTEND_MOTOR_SETPIONT);
    m_leftPIDController.setSetpoint(EXTEND_MOTOR_SETPIONT);
  }

  public void retract() {   
    m_rightPIDController.setSetpoint(RETRACT_MOTOR_SETPOINT);
    m_leftPIDController.setSetpoint(RETRACT_MOTOR_SETPOINT);
  }
 
public Command getExtend(){
  return runOnce(this::extend);
}
public Command getRetract(){
return runOnce(this::retract); 
}

}
  

  
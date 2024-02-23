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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climb.*;

public class ClimbSubsystem extends SubsystemBase{
  private CANSparkMax m_rightClimbMotor = new CANSparkMax(R_CLIMB_MOTOR_ID,MotorType.kBrushless);
  private CANSparkMax m_leftClimbMotor = new CANSparkMax(L_CLIMB_MOTOR_ID,MotorType.kBrushless);
  
  private PIDController m_rightPIDController = new PIDController(CLIMB_kP, CLIMB_kI, CLIMB_kD);
  private PIDController m_leftPIDController = new PIDController(CLIMB_kP, CLIMB_kI, CLIMB_kD);
      
  private RelativeEncoder m_rightClimbEncoder;
  private RelativeEncoder m_leftClimbEncoder;

  private double m_lastLeftCurrent = 0;
  private double m_lastRightCurrent = 0;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    m_rightClimbEncoder = m_rightClimbMotor.getEncoder();
    m_leftClimbEncoder = m_leftClimbMotor.getEncoder();
    m_rightClimbEncoder.setPosition(-EXTEND_MOTOR_SETPOINT);
    m_leftClimbEncoder.setPosition(EXTEND_MOTOR_SETPOINT);
    m_leftPIDController.setSetpoint(EXTEND_MOTOR_SETPOINT);
    m_rightPIDController.setSetpoint(-EXTEND_MOTOR_SETPOINT);
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

    
    m_rightClimbMotor.set(MathUtil.clamp(m_rightMotorOutput, MAX_DOWN_VOLTAGE,MAX_UP_VOLTAGE)); 
    m_leftClimbMotor.set(MathUtil.clamp(m_leftMotorOutput, -MAX_UP_VOLTAGE, -MAX_DOWN_VOLTAGE)); 

    //m_rightClimbMotor.set(0.1);
    //m_leftClimbMotor.set(-0.1);

    double newLeftCurrent = m_leftClimbMotor.getOutputCurrent();
    double newRightCurrent = m_rightClimbMotor.getOutputCurrent();

    SmartDashboard.putNumber("left position", m_leftClimbEncoder.getPosition());
    SmartDashboard.putNumber("right position", m_rightClimbEncoder.getPosition());
    SmartDashboard.putNumber("left output", m_leftMotorOutput);
    SmartDashboard.putNumber("right output", m_rightMotorOutput);
    SmartDashboard.putNumber("left setpoint", m_leftPIDController.getSetpoint());
    SmartDashboard.putNumber("right setpoint", m_rightPIDController.getSetpoint());
    // System.out.println(m_leftClimbMotor.getOutputCurrent());
    if (RobotState.isEnabled()) {
      //System.out.println((newLeftCurrent - m_lastLeftCurrent) + " " + (newRightCurrent - m_lastRightCurrent));
    }

    
    m_lastLeftCurrent = newLeftCurrent;
    m_lastRightCurrent = newRightCurrent;
    
}
  
  public void extend() {
    m_leftPIDController.setSetpoint(EXTEND_MOTOR_SETPOINT);
    m_rightPIDController.setSetpoint(-EXTEND_MOTOR_SETPOINT);
  }

  public void retract() {  
    m_leftPIDController.setSetpoint(RETRACT_MOTOR_SETPOINT);
    m_rightPIDController.setSetpoint(-RETRACT_MOTOR_SETPOINT); 
  }
 
public Command getExtendCommand(){
  return runOnce(this::extend);
}
public Command getRetractCommand(){
return runOnce(this::retract); 
}

}
  

  
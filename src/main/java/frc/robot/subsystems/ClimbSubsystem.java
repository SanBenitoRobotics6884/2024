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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ManualClimb;

import static frc.robot.Constants.Climb.*;

import java.util.function.BooleanSupplier;

public class ClimbSubsystem extends SubsystemBase{
  private CANSparkMax m_rightClimbMotor = new CANSparkMax(R_CLIMB_MOTOR_ID,MotorType.kBrushless);
  private CANSparkMax m_leftClimbMotor = new CANSparkMax(L_CLIMB_MOTOR_ID,MotorType.kBrushless);
  
  private PIDController m_rightPIDController = new PIDController(CLIMB_kP, CLIMB_kI, CLIMB_kD);
  private PIDController m_leftPIDController = new PIDController(CLIMB_kP, CLIMB_kI, CLIMB_kD);
      
  private RelativeEncoder m_rightClimbEncoder;
  private RelativeEncoder m_leftClimbEncoder;

  private boolean m_manualMode = false;

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
    m_rightClimbMotor.setSmartCurrentLimit(CLIMB_CURRENT_LIMIIT);
   m_leftClimbMotor.setSmartCurrentLimit(CLIMB_CURRENT_LIMIITS);
  }
   @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double m_rightCurrentHeight = m_rightClimbEncoder.getPosition();
    double m_leftCurrentHeight = m_leftClimbEncoder.getPosition();
    
    double m_rightMotorOutput = m_rightPIDController.calculate(m_rightCurrentHeight);
    double m_leftMotorOutput = m_leftPIDController.calculate(m_leftCurrentHeight);

    if (!m_manualMode) {
      m_rightClimbMotor.set(MathUtil.clamp(m_rightMotorOutput, MAX_DOWN_VOLTAGE, MAX_UP_VOLTAGE)); 
      m_leftClimbMotor.set(MathUtil.clamp(m_leftMotorOutput, -MAX_UP_VOLTAGE, -MAX_DOWN_VOLTAGE));   
    }

    SmartDashboard.putNumber("left position", m_leftClimbEncoder.getPosition());
    SmartDashboard.putNumber("right position", m_rightClimbEncoder.getPosition());
    SmartDashboard.putNumber("left output", m_leftMotorOutput);
    SmartDashboard.putNumber("right output", m_rightMotorOutput);
    SmartDashboard.putNumber("left setpoint", m_leftPIDController.getSetpoint());
    SmartDashboard.putNumber("right setpoint", m_rightPIDController.getSetpoint());
    
  }
    
  public void extend() {
    m_leftPIDController.setSetpoint(EXTEND_MOTOR_SETPOINT);
    m_rightPIDController.setSetpoint(-EXTEND_MOTOR_SETPOINT);
  }

  public void retract() {  
    m_leftPIDController.setSetpoint(RETRACT_MOTOR_SETPOINT);
    m_rightPIDController.setSetpoint(-RETRACT_MOTOR_SETPOINT); 
  }

  public void setManualMode(boolean isManualMode) {
    m_manualMode = isManualMode;
  }

  public void manual(boolean leftUp, boolean leftDown, boolean rightUp, boolean rightDown) {
    if (leftUp ^ leftDown) {
      m_leftClimbMotor.set(leftUp ? -0.1 : 0.1);
    } else {
      m_leftClimbMotor.set(0);
    }
    if (rightUp ^ rightDown) {
      m_rightClimbMotor.set(rightUp ? 0.1 : -0.1);
    } else {
      m_rightClimbMotor.set(0);
    }
  }

  public double getLeftVelocity() {
    return Math.abs(m_leftClimbEncoder.getVelocity());
  }

  public double getRightVelocity() {
    return Math.abs(m_rightClimbEncoder.getVelocity());
  }

  public void setMeasurementToZero() {
    m_leftClimbEncoder.setPosition(EXTEND_MOTOR_SETPOINT);
    m_rightClimbEncoder.setPosition(-EXTEND_MOTOR_SETPOINT);
  }
  
  public Command getExtendCommand(){
    return runOnce(this::extend);
  }

  public Command getRetractCommand(){
    return runOnce(this::retract); 
  }

  public Command getManualCommand(BooleanSupplier leftUp, BooleanSupplier leftDown, BooleanSupplier rightUp, BooleanSupplier rightDown) {
    return new ManualClimb(this, leftUp, leftDown, rightUp, rightDown);
  }

}
  

  
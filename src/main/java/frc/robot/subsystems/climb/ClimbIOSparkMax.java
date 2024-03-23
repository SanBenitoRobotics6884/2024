package frc.robot.subsystems.climb;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.Climb.L_CLIMB_MOTOR_ID;
import static frc.robot.Constants.Climb.R_CLIMB_MOTOR_ID;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbIOSparkMax implements ClimbIO { 
  private CANSparkMax m_rightClimbMotor = new CANSparkMax(R_CLIMB_MOTOR_ID,MotorType.kBrushless);
  private CANSparkMax m_leftClimbMotor = new CANSparkMax(L_CLIMB_MOTOR_ID,MotorType.kBrushless);
  private RelativeEncoder m_rightClimbEncoder = m_rightClimbMotor.getEncoder();
  private RelativeEncoder m_leftClimbEncoder = m_leftClimbMotor.getEncoder();

  @Override
  public void updateInputs(ClimbIOInputs inputs){
    inputs.leftPosition = m_leftClimbEncoder.getPosition();
    inputs.rightPosition = m_rightClimbEncoder.getPosition(); 
    inputs.leftCurrent = m_leftClimbMotor.getOutputCurrent(); 
    inputs.rightCurrent = m_rightClimbMotor.getOutputCurrent(); 
    inputs.leftTemperature = m_leftClimbMotor.getMotorTemperature();
    inputs.rightTemperature = m_rightClimbMotor.getMotorTemperature();
  }

  @Override
  public void setLeftDutyCycle(double percent){
    m_leftClimbMotor.set(percent);
  }

  @Override
  public void setRightDutyCycle(double percent){
    m_rightClimbMotor.set(percent);
  }

  @Override
  public void setLeftPosition(double position) {
    m_leftClimbEncoder.setPosition(position);
  }

  @Override
  public void setRightPosition(double position) {
    m_rightClimbEncoder.setPosition(position);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    m_leftClimbMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    m_rightClimbMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

}
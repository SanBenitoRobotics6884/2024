// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climb.*;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase{
  private ClimbIO m_io;

  private PIDController m_rightPIDController = new PIDController(CLIMB_kP, CLIMB_kI, CLIMB_kD);
  private PIDController m_leftPIDController = new PIDController(CLIMB_kP, CLIMB_kI, CLIMB_kD);

  private ClimbIOInputsAutoLogged m_inputs = new ClimbIOInputsAutoLogged();

  private boolean m_manualMode = false;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(ClimbIO io) {
    m_io = io;

    m_io.setRightPosition(-EXTEND_MOTOR_SETPOINT);
    m_io.setLeftPosition(EXTEND_MOTOR_SETPOINT);
    m_leftPIDController.setSetpoint(EXTEND_MOTOR_SETPOINT);
    m_rightPIDController.setSetpoint(-EXTEND_MOTOR_SETPOINT);
    m_io.setBrakeMode(true);
  }
   @Override
  public void periodic() { 
    m_io.updateInputs(m_inputs);
    Logger.processInputs("climb", m_inputs);

    double m_rightMotorOutput = m_rightPIDController.calculate(m_inputs.leftPosition);
    double m_leftMotorOutput = m_leftPIDController.calculate(m_inputs.rightPosition);

    if (!m_manualMode) {
      m_io.setRightDutyCycle(MathUtil.clamp(m_rightMotorOutput, MAX_DOWN_VOLTAGE, MAX_UP_VOLTAGE)); 
      m_io.setLeftDutyCycle(MathUtil.clamp(m_leftMotorOutput, -MAX_UP_VOLTAGE, -MAX_DOWN_VOLTAGE));   
    }

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
      m_io.setLeftDutyCycle(leftUp ? -0.1 : 0.1);
    } else {
      m_io.setLeftDutyCycle(0);
    }
    if (rightUp ^ rightDown) {
      m_io.setRightDutyCycle(rightUp ? 0.1 : -0.1);
    } else {
      m_io.setRightDutyCycle(0);
    }
  }

  public void setMeasurementToZero() {
    m_io.setLeftPosition(EXTEND_MOTOR_SETPOINT);
    m_io.setRightPosition(-EXTEND_MOTOR_SETPOINT);
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
  

  
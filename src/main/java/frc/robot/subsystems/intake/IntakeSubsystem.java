// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;


public class IntakeSubsystem extends SubsystemBase {

  private IntakeIO m_io;
  private IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

  private ProfiledPIDController m_pivotPID =
          new ProfiledPIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

  private boolean m_isZeroing = true;
 
  /** Creates a new Intake. */
  public IntakeSubsystem(IntakeIO io) {  
    m_io = io;  
    m_io.updateInputs(m_inputs);
    Logger.processInputs("intake", m_inputs);
    m_pivotPID.setTolerance(TOLERANCE);
    m_pivotPID.reset(m_inputs.pivotPosition, m_inputs.pivotVelocity);
    m_pivotPID.setGoal(STOW_SETPOINT);
  }

 @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("intake", m_inputs);
    double output = MathUtil.clamp(m_pivotPID.calculate(m_inputs.pivotPosition), -0.85, 0.85);
    boolean atZero = m_inputs.zeroLimitSwitch;
    if (m_isZeroing) {
      output = 0.1;
      if (atZero) {
        m_isZeroing = false;
        m_io.setPivotPosition(0);
      }
    }
    m_io.setPivotDutyCycle(output);
  }

  //We set the goal/setpoint to the PID
  public void deploy() {
    m_pivotPID.setGoal(DEPLOY_SETPOINT);
    
  }

  //Returns boolean of whather PID is at goal/setpoint.
  public boolean atSetpoint() {
    return m_pivotPID.atGoal();
    }

  
  // Set's speed to intake motor to reel
  public void reel() {
    m_io.setIntakeDutyCycle(INTAKE_MOTOR_REEL_SPEED);
  }

  // Returns whether limitswitch is triggered or not
  public boolean noteHeld() {
    return !m_inputs.noteLimitSwitch;
  }

  //Stops intake motor from spinning 
  public void rollerStop() {
    m_io.stopIntakeMotor();
  }

  public void spit() {
    m_io.setIntakeDutyCycle(INTAKE_MOTOR_SPIT_SPEED);
  }

  public void roll(double speed) {
    m_io.setIntakeDutyCycle(speed);
  }

  public void stow() {
    m_pivotPID.setGoal(STOW_SETPOINT);
  }

  public Command getReelCommand(BooleanSupplier cancel) {
    return new ReelCommand(this, cancel);
  }

  public Command getDeployCommand() {
    return new DeployIntakeCommand(this);
  }

  public Command getStowCommand() {
    return new StowIntakeCommand(this);
  }

  public Command getToSpeakerCommand() {
    return new IntakeToOuttakeWithDelay(this, INTAKE_MOTOR_SPEAKER_SPEED);
  }

  public Command getToAmpCommand() {
    return new IntakeToOuttakeWithDelay(this, INTAKE_MOTOR_AMP_SPEED);
  }

  public Command getEjectCommand() {
    return new IntakeToOuttake(this, INTAKE_MOTOR_EJECT_SPEED);
  }

  public Command getSuckCommand() {
    return new IntakeToOuttake(this, INTAKE_MOTOR_REEL_SPEED);
  }
  
}

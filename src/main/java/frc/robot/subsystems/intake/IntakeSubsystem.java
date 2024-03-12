// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;

import java.util.function.BooleanSupplier;


public class IntakeSubsystem extends SubsystemBase {

  

  private final CANSparkMax m_intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_pivotMotor = new CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
  
    //double kP = 0;
    //double kI = 0;    SET THE VARIABLES TO THE CONSTANTS. :fire: :skull:
    //double kD = 0;

  public ProfiledPIDController m_pivotPID =
          new ProfiledPIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
  private DigitalInput m_noteLimitSwitch = new DigitalInput(NOTE_LIMIT_SWITCH);
  private DigitalInput m_zeroLimitSwitch = new DigitalInput(ZERO_LIMIT_SWITCH);

  private RelativeEncoder m_pivotEncoder;

  private boolean m_isZeroing = true;;
 
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) DEPLOY_SETPOINT);
    m_pivotMotor.setSoftLimit(SoftLimitDirection.kReverse,(float) STOW_SETPOINT);

    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    // m_intakeMotor.setSmartCurrentLimit(0); UNNEEDED ATM

    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    // m_pivotMotor.setSmartCurrentLimit(0); UNNEEDED ATM
    m_pivotEncoder = m_pivotMotor.getEncoder();
    m_pivotEncoder.setPosition(ENCODER_POSITION);
    
    m_pivotPID.setTolerance(TOLERANCE);

    m_pivotPID.reset(m_pivotEncoder.getPosition(), m_pivotEncoder.getVelocity());
  }

 @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double output = MathUtil.clamp(m_pivotPID.calculate(m_pivotEncoder.getPosition()), -0.85, 0.85);
    boolean atZero = m_zeroLimitSwitch.get();
    if (m_isZeroing) {
      output = 0.1;
      if (atZero) {
        m_isZeroing = false;
        m_pivotEncoder.setPosition(0);
      }
    }
    m_pivotMotor.set(output);

    SmartDashboard.putNumber("pivot position", m_pivotEncoder.getPosition());
    // SmartDashboard.putNumber("pivot setpoint", m_pivotPID.getSetpoint().position);
    // SmartDashboard.putNumber("pivot goal", m_pivotPID.getGoal().position);
    // SmartDashboard.putNumber("pivot output", output);
    SmartDashboard.putBoolean("pivot zero limit switch", atZero);
    SmartDashboard.putBoolean("note limit switch", m_noteLimitSwitch.get());
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
    m_intakeMotor.set(INTAKE_MOTOR_REEL_SPEED);
  }

  // Returns whether limitswitch is triggered or not
  public boolean noteHeld() {
    return !m_noteLimitSwitch.get();
  }

  //Stops intake motor from spinning 
  public void rollerStop() {
      m_intakeMotor.stopMotor();
  }

  public void spit() {
    m_intakeMotor.set(INTAKE_MOTOR_SPIT_SPEED);
  }

  public void roll(double speed) {
    m_intakeMotor.set(speed);
  }

  public boolean noteGone() {
    if (m_noteLimitSwitch.get() == true) {
      return false;
    } else if(m_noteLimitSwitch.get() == false) {
      return true;
    }
    return false;
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

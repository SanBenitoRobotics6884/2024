// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;


public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_pivotMotor = new CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
  
    //double kP = 0;
    //double kI = 0;    SET THE VARIABLES TO THE CONSTANTS. :fire: :skull:
    //double kD = 0;

  private ProfiledPIDController m_pivotPID =
     new ProfiledPIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
  private  DigitalInput m_limitSwitch = new DigitalInput(0);

  RelativeEncoder m_intakeEncoder;
  RelativeEncoder m_pivotEncoder;
 
  /** Creates a new Intake. */
  public IntakeSubsystem() {

    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //m_intakeMotor.setSmartCurrentLimit(0); UNNEEDED ATM
    m_intakeEncoder = m_intakeMotor.getEncoder();

    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //m_pivotMotor.setSmartCurrentLimit(0); UNNEEDED ATM
    m_pivotEncoder = m_pivotMotor.getEncoder();
    m_pivotEncoder.setPosition(ENCODER_POSITION);
    
    m_pivotPID.setTolerance(TOLERANCE);
        /**
      Saw this in cranberry code, it sets like current limit in AMPs though, I don't know what it is so LUCAS help.
    */

    //dunno if this is useful atm but we'll see
  }

 @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if (m_pivotPID.atGoal()) {
      m_pivotMotor.setVoltage(0);
    } else {
      m_pivotMotor.set(m_pivotPID.calculate(m_pivotEncoder.getPosition(), GOAL));
    }
  }

  //We set the goal/setpoint to the PID
  public void deployGoal() {
    m_pivotPID.setGoal(GOAL);
    
  }

  //Returns boolean of whather PID is at goal/setpoint.
  public boolean deployHitGoal() {
    return m_pivotPID.atGoal();
    }

  
  // Set's speed to intake motor to reel
  public void reel() {
    m_intakeMotor.set(INTAKE_MOTOR_SPEED);
  }

  // Returns whether limitswitch is triggered or not
  public boolean noteHeld() {
    return m_limitSwitch.get();
  }

  //Stops intake motor from spinning 
  public void reelStop() {
      m_intakeMotor.stopMotor();
  }

  public void spit() {
    m_intakeMotor.set(INTAKE_MOTOR_SPIT_SPEED);
  }

  
  
}

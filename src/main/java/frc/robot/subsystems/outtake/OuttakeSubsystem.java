// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Outtake.*;

import org.littletonrobotics.junction.Logger;

public class OuttakeSubsystem extends SubsystemBase {
  OuttakeIO m_io;
  OuttakeIOInputsAutoLogged m_inputs = new OuttakeIOInputsAutoLogged();

  PIDController m_PID = new PIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD);

  double m_pivotSetpoint = 0;
  boolean m_isZeroing = true; // Should initially be true

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem(OuttakeIO io) {
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("outtake", m_inputs);

    if (m_isZeroing) {
      m_io.setPivotDutyCycle(ZEROING_VOLTAGE);
      if (m_inputs.speakerLimitSwitch) {
        m_isZeroing = false;
        m_io.setPivotPosition(0);
      }
    } else {
      m_io.setPivotDutyCycle(m_PID.calculate(m_inputs.pivotPosition, m_pivotSetpoint));
    }

  }

  public boolean isInAmpPosition() {
    return m_inputs.pivotPosition < (SPEAKER_POSITION + AMP_POSITION) / 2.0;
  }

  public boolean isNotZeroing() {
    return !m_isZeroing;
  }

  public void toSpeakerPosition() {
    setSetpoint(SPEAKER_POSITION);
  }

  public void toAmpPosition() {
    setSetpoint(AMP_POSITION);
  }

  public void setSetpoint(double setpoint) {
    m_pivotSetpoint = setpoint;
  }

  public boolean atSetpoint() {
    return Math.abs(m_inputs.pivotPosition - m_pivotSetpoint) < TOLERANCE;
  }

  public void rollOuttake(double takeNoteSpeed) {
    m_io.setPassOffDutyCycle(takeNoteSpeed);
  }

  public void stopMotors() {
    m_io.stopPassOffMotor();
  }

  // The following code it is not used, but we love it, so we're leaving it here. :fire: :skull:

  public Command shootToAmpCommand() {
    return run(() -> rollOuttake(AMP_PERCENT_OUTPUT)).finallyDo(this::stopMotors);
  }

  public Command shootToSpeakerCommand() {
    return run(() -> rollOuttake(SPEAKER_PERCENT_OUTPUT)).finallyDo(this::stopMotors);
  }

  public Command yoinkNoteCommand() {
    return run(() -> rollOuttake(YOINK_PERCENT_OUTPUT)).finallyDo(this::stopMotors);
  }

  // The two following commands make the robot outtake to rotate either amp or speaker possition. :fire: :sob:

  public Command rotateToAmpPositionCommand() {
    return new FunctionalCommand(
      this::toAmpPosition, 
      () -> {}, 
      b -> {}, 
      this::atSetpoint, 
      this);
  }

  public Command rotateToSpeakerCommand() {
    return new FunctionalCommand(
      this::toSpeakerPosition, 
      () -> {}, 
      b -> {}, 
      this::atSetpoint, 
      this);
  }
}

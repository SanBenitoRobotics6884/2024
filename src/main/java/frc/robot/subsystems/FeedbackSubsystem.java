// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriverFeedBack.*;

public class FeedbackSubsystem extends SubsystemBase {
  AddressableLED m_Led = new AddressableLED(0);
  AddressableLEDBuffer m_Buffer = new AddressableLEDBuffer(LEDS_LENGTH);
  BooleanSupplier m_hasNote;

  /** Creates a new FeedBack_Subsystem. */
  public FeedbackSubsystem(BooleanSupplier hasNote) {
    m_hasNote = hasNote;

    m_Led.setData(m_Buffer);
    m_Led.setLength(m_Buffer.getLength());
    m_Led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void noteIn() {
    for (var i = 0; i < m_Buffer.getLength(); i++) {
      m_Buffer.setRGB(i, 225, 112, 0);
    }
  }
}

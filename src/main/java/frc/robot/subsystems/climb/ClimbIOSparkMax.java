package frc.robot.subsystems.climb;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import static frc.robot.Constants.Climb.L_CLIMB_MOTOR_ID;
import static frc.robot.Constants.Climb.R_CLIMB_MOTOR_ID;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbIOSparkMax implements ClimbIO {
    
    private CANSparkMax m_rightClimbMotor = new CANSparkMax(R_CLIMB_MOTOR_ID,MotorType.kBrushless);
    private CANSparkMax m_leftClimbMotor = new CANSparkMax(L_CLIMB_MOTOR_ID,MotorType.kBrushless);
    private RelativeEncoder m_rightClimbEncoder = m_rightClimbMotor.getEncoder();
    private RelativeEncoder m_leftClimbEncoder = m_leftClimbMotor.getEncoder();

    @Override
    public void  setleftAppliedVoltage(double volts ){
        m_leftClimbMotor.set(volts);
    }

    public void  setrightAppliedVoltage(double volts){
        m_rightClimbMotor.set(volts);
    }

    public void updateInputs(ClimbIOInputs Inputs){
        Inputs.leftposition = m_leftClimbEncoder.getPosition();
        Inputs.rightposition = m_rightClimbEncoder.getPosition(); 
        Inputs.leftvoltage = m_leftClimbMotor.getAppliedOutput() * m_leftClimbMotor.getBusVoltage(); 
        Inputs.rightvoltage = m_rightClimbMotor.getAppliedOutput() * m_rightClimbMotor.getBusVoltage(); 
        Inputs.m_leftCurrent = m_leftClimbMotor.getOutputCurrent(); 
        Inputs.m_rightCurrent = m_rightClimbMotor.getOutputCurrent(); 
    }
    }

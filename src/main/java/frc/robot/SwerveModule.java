package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.Swerve.*;

public class SwerveModule { 
  private TalonFX m_driveMotor;
  private CANSparkMax m_steerMotor;

  private CANcoder m_steerAbsoluteEncoder;
  private RelativeEncoder m_steerIntegratedEncoder;

  private Rotation2d m_angleReference;
  private double m_velocityReference; // meters per second

  private VelocityVoltage m_request;

  public SwerveModule(int driveID, int steerID, int encoderID, boolean driveInverted, 
                      boolean steerInverted, double magnetOffset) {
    m_driveMotor = new TalonFX(driveID); 
    m_steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);

    m_driveMotor.setInverted(driveInverted);
    m_steerMotor.setInverted(steerInverted);

    m_steerAbsoluteEncoder = new CANcoder(encoderID);
    MagnetSensorConfigs config = new MagnetSensorConfigs();
    config.MagnetOffset = magnetOffset;
    config.SensorDirection = steerInverted ? 
        SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
    config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf                ;
    m_steerAbsoluteEncoder.getConfigurator().apply(config);

    m_steerIntegratedEncoder = m_steerMotor.getEncoder();
    m_steerIntegratedEncoder.setPositionConversionFactor(STEER_POSITION_CONVERSION);
    m_steerIntegratedEncoder.setPosition(m_steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    
    SparkPIDController steerController = m_steerMotor.getPIDController();
    steerController.setPositionPIDWrappingMinInput(0);
    steerController.setPositionPIDWrappingMaxInput(1.0); // rotations
    steerController.setPositionPIDWrappingEnabled(true);
    steerController.setP(STEER_kP);
    steerController.setI(STEER_kI);
    steerController.setD(STEER_kD);
    m_steerMotor.setClosedLoopRampRate(STEER_RAMP_RATE);

   
     var Slot0Configs = new Slot0Configs(); 
    Slot0Configs.kP = 0; 
    Slot0Configs.kI = 0; 
    Slot0Configs.kD = 0; 
    Slot0Configs.kS = 0.03; 
   Slot0Configs.kV = 0.09; 
   m_driveMotor.getConfigurator().apply(Slot0Configs);
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0); 
    m_driveMotor.setControl(m_request.withPosition(0));
    m_angleReference = new Rotation2d();
    m_driveMotor.setControl(m_request);
    
  }
    
  public void setState(SwerveModuleState state) {
    m_angleReference = state.angle;
    m_velocityReference = state.speedMetersPerSecond;
    
    m_steerMotor.getPIDController().setReference(m_angleReference.getRotations(), ControlType.kPosition);
     m_request = new VelocityVoltage(m_velocityReference); 
    m_driveMotor.setControl(m_request);
  }
 public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(m_driveMotor.getPosition().getValueAsDouble(), getRotation2d());
  }
 
  public void setIntegratedEncoderPositionToAbsoluteEncoderMeasurement() {
    m_steerIntegratedEncoder.setPosition(m_steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(m_steerIntegratedEncoder.getPosition());
  }

  public Rotation2d getDesiredRotation2d() {
    return m_angleReference;
  }

 

  public double getDriveEncoderPosition() {
    return m_driveMotor.getPosition().getValueAsDouble();
  }

  public double getAngleDegrees() {
    return Units.rotationsToDegrees(m_steerIntegratedEncoder.getPosition());
  }

  public double getDesiredAngleDegrees() {
    return m_angleReference.getDegrees();
  }

  public double getVelocity() {
    return m_driveMotor.getVelocity().getValueAsDouble();
  }

  public double getDesiredVelocity() {
    return m_velocityReference;
  }
   
  public void putData(String name) {
    SmartDashboard.putNumber(name + " cancoder rot", m_steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber(name + " neo rot", m_steerIntegratedEncoder.getPosition());
    SmartDashboard.putNumber(name + " desired rot", m_angleReference.getRotations());
    /**
    SmartDashboard.putNumber(name + " vel", getVelocity());
    SmartDashboard.putNumber(name + " desired vel", getDesiredVelocity());
    */
  }

}

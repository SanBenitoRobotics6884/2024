package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.Swerve.*;

public class SwerveModule {

  private CANSparkMax m_driveMotor;
  private CANSparkMax m_steerMotor;

  private CANcoder m_steerAbsoluteEncoder;
  private RelativeEncoder m_steerIntegratedEncoder;
  private RelativeEncoder m_driveEncoder;

  private SimpleMotorFeedforward m_driveFeedforward;

  private Rotation2d m_angleReference;
  private double m_velocityReference; // meters per second
    
  public SwerveModule(int driveID, int steerID, int encoderID, boolean driveInverted, 
                      boolean steerInverted, double magnetOffset) {
    m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
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

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPosition(0);
    m_driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION);
    m_driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION);
    
    SparkPIDController steerController = m_steerMotor.getPIDController();
    steerController.setPositionPIDWrappingMinInput(0);
    steerController.setPositionPIDWrappingMaxInput(1.0); // rotations
    steerController.setPositionPIDWrappingEnabled(true);
    steerController.setP(STEER_kP);
    steerController.setI(STEER_kI);
    steerController.setD(STEER_kD);
    m_steerMotor.setClosedLoopRampRate(STEER_RAMP_RATE);

    SparkPIDController driveController = m_driveMotor.getPIDController();
    driveController.setP(DRIVE_kP);
    driveController.setI(DRIVE_kI);
    driveController.setD(DRIVE_kD);
    m_driveMotor.setClosedLoopRampRate(DRIVE_RAMP_RATE);

    m_driveFeedforward = new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV);

    m_angleReference = new Rotation2d();
  }

  public void setState(SwerveModuleState state) {
    m_angleReference = state.angle;
    m_velocityReference = state.speedMetersPerSecond;
    
    m_steerMotor.getPIDController().setReference(m_angleReference.getRotations(), ControlType.kPosition);
    m_driveMotor.getPIDController().setReference(
        m_velocityReference, ControlType.kVelocity, 0, 
        m_driveFeedforward.calculate(m_velocityReference));
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

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), getRotation2d());
  }

  public double getDriveEncoderPosition() {
    return m_driveEncoder.getPosition();
  }

  public double getAngleDegrees() {
    return Units.rotationsToDegrees(m_steerIntegratedEncoder.getPosition());
  }

  public double getDesiredAngleDegrees() {
    return m_angleReference.getDegrees();
  }

  public double getVelocity() {
    return m_driveEncoder.getVelocity();
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

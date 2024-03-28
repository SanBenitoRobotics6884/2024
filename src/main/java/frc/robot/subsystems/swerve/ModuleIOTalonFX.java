package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;

import static frc.robot.Constants.Swerve.*;

public class ModuleIOTalonFX implements ModuleIO { 
  private TalonFX m_driveMotor;
  private CANSparkMax m_steerMotor;

  private CANcoder m_steerAbsoluteEncoder;
  private RelativeEncoder m_steerIntegratedEncoder;

  private VelocityVoltage m_request = new VelocityVoltage(0);

  private StatusSignal<Double> m_absoluteAngle;

  public ModuleIOTalonFX(int driveID, int steerID, int encoderID, boolean driveInverted, 
                      boolean steerInverted, double magnetOffset) {
    m_driveMotor = new TalonFX(driveID); 
    m_steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);

    m_driveMotor.setInverted(driveInverted);
    m_steerMotor.setInverted(steerInverted);

    m_steerAbsoluteEncoder = new CANcoder(encoderID);
    MagnetSensorConfigs cancoderConfigs = new MagnetSensorConfigs();
    cancoderConfigs.MagnetOffset = magnetOffset;
    cancoderConfigs.SensorDirection = steerInverted ? 
        SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
    cancoderConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf                ;
    m_steerAbsoluteEncoder.getConfigurator().apply(cancoderConfigs);
    m_absoluteAngle = m_steerAbsoluteEncoder.getAbsolutePosition();
    
    m_absoluteAngle.setUpdateFrequency(4);
    m_steerAbsoluteEncoder.optimizeBusUtilization();

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

    var drivePIDFConfigs = new Slot0Configs(); 
    drivePIDFConfigs.kP = DRIVE_kP; 
    drivePIDFConfigs.kI = DRIVE_kI; 
    drivePIDFConfigs.kD = DRIVE_kD; 
    drivePIDFConfigs.kS = DRIVE_kS; 
    drivePIDFConfigs.kV = DRIVE_kV; 
    var driveRampConfigs = new ClosedLoopRampsConfigs();
    driveRampConfigs.VoltageClosedLoopRampPeriod = DRIVE_RAMP_RATE;
    var driveCurrentLimitConfigs = new CurrentLimitsConfigs();
    driveCurrentLimitConfigs.StatorCurrentLimit = DRIVE_CURRENT_LIMIT;
    driveCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    var driveConfigurator = m_driveMotor.getConfigurator();
    driveConfigurator.apply(new TalonFXConfiguration());
    driveConfigurator.apply(drivePIDFConfigs);
    driveConfigurator.apply(driveRampConfigs);
    driveConfigurator.apply(driveCurrentLimitConfigs);

    m_driveMotor.optimizeBusUtilization();
    m_driveMotor.getPosition().setUpdateFrequency(50);
    m_driveMotor.getVelocity().setUpdateFrequency(50);
    m_driveMotor.getStatorCurrent().setUpdateFrequency(50);
    m_driveMotor.getDutyCycle().setUpdateFrequency(50);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.absoluteAngle = Rotation2d.fromRotations(m_absoluteAngle.refresh().getValueAsDouble());
    inputs.relativeAngle = Rotation2d.fromRotations(m_steerIntegratedEncoder.getPosition());
    inputs.steerCurrent = m_steerMotor.getOutputCurrent();
    inputs.steerDutyCycle = m_steerMotor.getAppliedOutput();

    inputs.drivePosition = m_driveMotor.getRotorPosition().getValueAsDouble() * DRIVE_POSITION_CONVERSION;
    inputs.driveVelocity = m_driveMotor.getRotorVelocity().getValueAsDouble() * DRIVE_POSITION_CONVERSION;
    inputs.driveCurrent = m_driveMotor.getStatorCurrent().getValueAsDouble();
    inputs.driveDutyCycle = m_driveMotor.getDutyCycle().getValueAsDouble();
  }
  
  @Override
  public void setState(SwerveModuleState state) {
    m_request.Velocity = state.speedMetersPerSecond / DRIVE_POSITION_CONVERSION;
    m_steerMotor.getPIDController().setReference(state.angle.getRotations(), ControlType.kPosition);
    m_driveMotor.setControl(m_request);
  }
  
  @Override
  public void resetToAbsolute() {
    m_steerIntegratedEncoder.setPosition(m_steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
  }

}

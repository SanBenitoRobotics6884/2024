package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.Swerve.*;

public class ModuleIOSparkMax implements ModuleIO {
  private CANSparkMax m_driveMotor;
  private CANSparkMax m_steerMotor;

  private CANcoder m_steerAbsoluteEncoder;
  private RelativeEncoder m_steerIntegratedEncoder;
  private RelativeEncoder m_driveEncoder;

  private StatusSignal<Double> m_absoluteAngle;
  
  public ModuleIOSparkMax(int driveId, int steerId, int encoderId, boolean driveInverted,
                          boolean steerInverted, double magnetOffset) {
    m_driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
    m_steerMotor = new CANSparkMax(steerId, MotorType.kBrushless);

    m_driveMotor.setInverted(driveInverted);
    m_steerMotor.setInverted(steerInverted);
    m_driveMotor.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
    m_steerMotor.setSmartCurrentLimit(STEER_CURRENT_LIMIT);

    m_steerAbsoluteEncoder = new CANcoder(encoderId);
    MagnetSensorConfigs config = new MagnetSensorConfigs();
    config.MagnetOffset = magnetOffset;
    config.SensorDirection = steerInverted ? 
        SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
    config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    m_steerAbsoluteEncoder.getAbsolutePosition().setUpdateFrequency(10);
    m_steerAbsoluteEncoder.optimizeBusUtilization();               ;
    m_steerAbsoluteEncoder.getConfigurator().apply(config);
    m_absoluteAngle = m_steerAbsoluteEncoder.getAbsolutePosition();

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

    SparkPIDController driveController = m_driveMotor.getPIDController();
    driveController.setP(DRIVE_kP);
    driveController.setI(DRIVE_kI);
    driveController.setD(DRIVE_kD);
    m_driveMotor.setClosedLoopRampRate(DRIVE_RAMP_RATE);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.absoluteAngle = Rotation2d.fromRotations(m_absoluteAngle.getValueAsDouble());
    inputs.relativeAngle = Rotation2d.fromRotations(m_steerIntegratedEncoder.getPosition());
    inputs.steerCurrent = m_steerMotor.getOutputCurrent();
    inputs.steerDutyCycle = m_steerMotor.getAppliedOutput();

    inputs.drivePosition = m_driveEncoder.getPosition();
    inputs.driveVelocity = m_driveEncoder.getVelocity();
    inputs.driveCurrent = m_driveMotor.getOutputCurrent();
    inputs.driveDutyCycle = m_driveMotor.getAppliedOutput();
  }

  @Override
  public void setState(SwerveModuleState state) {
    m_steerMotor.getPIDController().setReference(state.angle.getRotations(), ControlType.kPosition);
    m_driveMotor.getPIDController().setReference(
        state.speedMetersPerSecond, ControlType.kVelocity, 0, 
        DRIVE_FEEDFORWARD.calculate(state.speedMetersPerSecond));
  }

  @Override
  public void resetToAbsolute() {
    m_steerIntegratedEncoder.setPosition(m_steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
  }

}

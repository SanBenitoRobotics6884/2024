package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
  
  @AutoLog
  public static class ModuleIOInputs {
    public Rotation2d absoluteAngle = new Rotation2d();
    public Rotation2d relativeAngle = new Rotation2d();
    public double steerCurrent = 0;
    public double steerDutyCycle = 0;

    public double drivePosition = 0;
    public double driveVelocity = 0;
    public double driveCurrent = 0;
    public double driveDutyCycle = 0;
  }

  public void updateInputs(ModuleIOInputs inputs);

  public void setState(SwerveModuleState state);

  public void resetToAbsolute();

}

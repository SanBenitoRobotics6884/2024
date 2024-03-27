package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
  
  @AutoLog
  public static class VisionIOInputs {
    public Pose3d estimatedPose = null;
    public double timestampSeconds = 0;
    public boolean isConnected = false;

    public int[] fiducialIds = new int[0];
  }

  public void updateInputs(VisionIOInputs inputs);

  public void takeSnapshot();

}

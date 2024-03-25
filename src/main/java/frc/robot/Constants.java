package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {

  public static final class Swerve {
    public static final double STEER_kP = 3.0;
    public static final double STEER_kI = 0;
    public static final double STEER_kD = 0;
    public static final double STEER_RAMP_RATE = 0.15; // how many seconds to go from 0 to full throttle
    public static final int STEER_CURRENT_LIMIT = 20;

    public static final double DRIVE_kP = 0.2;
    public static final double DRIVE_kI = 0;
    public static final double DRIVE_kD = 0.05;
    public static final double DRIVE_kS = 0.11;
    public static final double DRIVE_kV = 3.0;
    public static final double DRIVE_RAMP_RATE = 0.15;

    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = 
        new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV);
    public static final int DRIVE_CURRENT_LIMIT = 40;
   
    public static final double MAX_OUTPUT = 0.3;

    public static final boolean SQUARED_INPUTS = false; // at SVR, this was false

    public static final double WHEEL_RADIUS = 2.0; // inches, need to double check
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;
    public static final double DRIVE_POSITION_CONVERSION = 
        2.0 * Math.PI * Units.inchesToMeters(WHEEL_RADIUS) / DRIVE_GEAR_RATIO; // meters per rotation
    public static final double DRIVE_VELOCITY_CONVERSION = 
        DRIVE_POSITION_CONVERSION / 60.0; // meters per second
    public static final double STEER_POSITION_CONVERSION = 1 / STEER_GEAR_RATIO; // rotations

    public static final int BR_DRIVE_ID = 1;
    public static final int FR_DRIVE_ID = 2;
    public static final int BL_DRIVE_ID = 3;
    public static final int FL_DRIVE_ID = 4;

    public static final int BR_STEER_ID = 5;
    public static final int FR_STEER_ID = 6;
    public static final int BL_STEER_ID = 7;
    public static final int FL_STEER_ID = 8;

    public static final int BR_ENCODER_ID = 9;
    public static final int FR_ENCODER_ID = 10;
    public static final int BL_ENCODER_ID = 11;
    public static final int FL_ENCODER_ID = 12;

    public static final int PIGEON_ID = 13;

    public static final boolean FR_DRIVE_INVERTED = false;
    public static final boolean FL_DRIVE_INVERTED = true;
    public static final boolean BR_DRIVE_INVERTED = false;
    public static final boolean BL_DRIVE_INVERTED = false;

    public static final boolean FR_STEER_INVERTED = true;
    public static final boolean FL_STEER_INVERTED = true;
    public static final boolean BR_STEER_INVERTED = true;
    public static final boolean BL_STEER_INVERTED = true;

    public static final double FR_OFFSET_ROTATIONS = 0.1644;
    public static final double FL_OFFSET_ROTATIONS = 0.5406;
    public static final double BR_OFFSET_ROTATIONS = 0.9320;
    public static final double BL_OFFSET_ROTATIONS = 0.2593;

    public static final double APOTHEM = Units.inchesToMeters(10.625); // outdated
    public static final Translation2d FR_LOCATION = new Translation2d(APOTHEM, -APOTHEM);
    public static final Translation2d FL_LOCATION = new Translation2d(APOTHEM, APOTHEM);
    public static final Translation2d BR_LOCATION = new Translation2d(-APOTHEM, -APOTHEM);
    public static final Translation2d BL_LOCATION = new Translation2d(-APOTHEM, APOTHEM);

    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = 
        new HolonomicPathFollowerConfig(
            new PIDConstants(2,0,0),
            new PIDConstants(2, 0, 0),
            3.5,
            0.4445,
            new ReplanningConfig());

  }
    
  public static final class Climb {
    public static final double MAX_UP_VOLTAGE = 0.15; 
    public static final double MAX_DOWN_VOLTAGE = -0.8;
    public static final double EXTEND_MOTOR_SETPOINT = 0; // this is where climber starts
    public static final double RETRACT_MOTOR_SETPOINT = 85.0;
    public static final int CLIMB_CURRENT_LIMIT = 40; 
    public static final int R_CLIMB_MOTOR_ID = 14;
    public static final int L_CLIMB_MOTOR_ID = 15;
    public static final double CLIMB_kP = 0.05;
    public static final double CLIMB_kI = 0;
    public static final double CLIMB_kD = 0;    

    public static final double ZEROING_SPEED = 0.1;
    public static final double VELOCITY_THRESHOLD = 10.0;
  }

  public static final class Outtake {
    public static final int PASS_OFF_MOTOR_ID = 16;
    public static final int PIVOT_MOTOR_ID = 19;

    public static final int AMP_LIMIT_SWITCH_CHANNEL = 3;

    public static final double PIVOT_kP = 0.1;
    public static final double PIVOT_kI = 0;
    public static final double PIVOT_kD = 0;

    public static final double SPEAKER_POSITION = -1.25;
    public static final double AMP_POSITION = -5.0; 

    public static final double ZEROING_VOLTAGE = 0.05; 

    public static final double SPEAKER_PERCENT_OUTPUT = 0.8;
    public static final double AMP_PERCENT_OUTPUT = 0.3;
    public static final double YOINK_PERCENT_OUTPUT = 0.3;

    public static final double TOLERANCE = 1.00;

    public static final int PASS_OFF_CURRENT_LIMIT = 40; 
    public static final int PIVOT_CURRENT_LIMIT = 60;
  }

  public final static class Intake {
    public static final double PIVOT_kP = 0.15;
    public static final double PIVOT_kI = 0;
    public static final double PIVOT_kD = 0;

    public static final int INTAKE_MOTOR_ID = 20;
    public static final int PIVOT_MOTOR_ID = 21;
    public static final int NOTE_LIMIT_SWITCH = 2;
    public static final int ZERO_LIMIT_SWITCH = 1;

    public static final double INTAKE_MOTOR_REEL_SPEED = -0.8;
    public static final double PIVOT_MOTOR_SPEED = 0.1;
    public static final double INTAKE_MOTOR_SPIT_SPEED = 0.8;
    public static final double INTAKE_MOTOR_AMP_SPEED = 0.4;
    public static final double INTAKE_MOTOR_EJECT_SPEED = 0.4;
    public static final double INTAKE_MOTOR_SPEAKER_SPEED = 0.8;

    public static final double MAX_VELOCITY = 66.7;
    public static final double MAX_ACCELERATION = 30;

    public static final double TOLERANCE = 5.0;
    
    public static final double DEPLOY_SETPOINT = -91.2;
    public static final double STOW_SETPOINT = -4.5;
    
    public static final double ZEROING_SPEED = 0.1;

    public static final double ENCODER_POSITION = 0;

    public static final int INTAKE_CURRENT_LIMIT = 40;
    public static final int PIVOT_CURRENT_LIMIT = 40;
  }

  public static final class Vision {
    public static final double POSE_HEIGHT_THRESHOLD = 0.5;
    public static final double POSE_ANGLE_THRESHOLD = 10;

    public static final AprilTagFieldLayout LAYOUT = 
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
    public static final CameraConstants SHOOTER_CAMERA =
        new CameraConstants("ov9281", new Transform3d(
            -0.2667, -0.1397, 0.51562, 
            new Rotation3d(0, -Math.PI / 12.0, Math.PI)));
  }

  public static final class CameraConstants {
    public final Transform3d robotToCamera;
    public final String cameraName;
    
    public CameraConstants(String cameraName, Transform3d robotToCamera) {
      this.cameraName = cameraName;
      this.robotToCamera = robotToCamera;
    }
  }

  public static final class Field {
    public static final Pose2d SPEAKER_SHOOTING_POSE;
    public static final Pose2d AMP_SHOOTING_POSE;
    public static final Pose2d SOURCE_GETTING_POSE;
    public static final Translation2d SPEAKER;

    static {
      Pose2d speakerShootingPose = new Pose2d(new Translation2d(1.839, 5.601), Rotation2d.fromDegrees(0));
      Pose2d ampShootingPose = new Pose2d(new Translation2d(1.719, 7.236), Rotation2d.fromDegrees(90));
      Pose2d sourceGettingPose = new Pose2d(new Translation2d(14.403, 2.580), Rotation2d.fromDegrees(-60));
      Pose2d speaker = Vision.LAYOUT.getTagPose(7).get().toPose2d();
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        speakerShootingPose = GeometryUtil.flipFieldPose(speakerShootingPose);
        ampShootingPose = GeometryUtil.flipFieldPose(ampShootingPose);
        sourceGettingPose = GeometryUtil.flipFieldPose(sourceGettingPose);
        speaker = GeometryUtil.flipFieldPose(speaker);
      }
      SPEAKER_SHOOTING_POSE = speakerShootingPose;
      AMP_SHOOTING_POSE = ampShootingPose;
      SOURCE_GETTING_POSE = sourceGettingPose;
      SPEAKER = speaker.getTranslation();
    }

    public static double getDistanceMetersToSpeaker(Pose2d pose) {
      return pose.getTranslation().getDistance(SPEAKER);
    }

    public static double getAngleDegreesToSpeaker(Pose2d pose) {
      return pose.getTranslation().minus(SPEAKER).getAngle().getDegrees();
    }

  }

  public static final class ShootingData {
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_OUTTAKE_SETPOINT = 
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap OUTTAKE_TO_INTAKE_SETPOINT =
        new InterpolatingDoubleTreeMap();

    static {
      double[] distanceToOuttakeSetpointData = new double[] {
          1.0, -1.25};
      double[] outtakeToIntakeSetpointData = new double[] {
          -1.25, -4.5};

      for (int i = 0; i < distanceToOuttakeSetpointData.length; i += 2) {
        DISTANCE_TO_OUTTAKE_SETPOINT.put(
            distanceToOuttakeSetpointData[i], distanceToOuttakeSetpointData[i + 1]);
      }
      for (int i = 0; i < outtakeToIntakeSetpointData.length; i += 2) {
        OUTTAKE_TO_INTAKE_SETPOINT.put(
            outtakeToIntakeSetpointData[i], outtakeToIntakeSetpointData[i + 1]);
      }

    }
    
  }
  
}

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class Swerve {
    public static final double STEER_kP = 3.0;
    public static final double STEER_kI = 0;
    public static final double STEER_kD = 0;
    public static final double STEER_RAMP_RATE = 0.15; // how many seconds to go from 0 to full throttle
    public static final int STEER_CURRENT_LIMIT = 20;

    public static final double DRIVE_kP = 0.2;
    public static final double DRIVE_kI = 0;
    public static final double DRIVE_kD = 0;
    public static final double DRIVE_kS = 0.11;
    public static final double DRIVE_kV = 3.0;
    public static final double DRIVE_RAMP_RATE = 0.15;

    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = 
        new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV);
    public static final int DRIVE_CURRENT_LIMIT = 40;
   
    public static final double MAX_OUTPUT = 0.3;

    public static final boolean SQUARED_INPUTS = false;

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

    public static final int AMP_LIMIT_SWITCH_CHANNEL = 0;

    public static final double PIVOT_kP = 2.0;
    public static final double PIVOT_kI = 0;
    public static final double PIVOT_kD = 0;

    public static final double SPEAKER_POSITION = -0.305; // We need to set these again! 3-23
    public static final double AMP_POSITION = -0.22; 

    public static final double ZEROING_VOLTAGE = -0.15; // Before 3-23 it was positive (now, the limit switch is on the speaker side)

    public static final double SPEAKER_PERCENT_OUTPUT = 0.8;
    public static final double AMP_PERCENT_OUTPUT = 0.3;
    public static final double YOINK_PERCENT_OUTPUT = 0.3;

    public static final double TOLERANCE = 0.05;

    public static final int PASS_OFF_CURRENT_LIMIT = 40; 
    public static final int PIVOT_CURRENT_LIMIT = 40;
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
    
    public static final double DEPLOY_SETPOINT = -91.5;
    public static final double STOW_SETPOINT = -2.0;
    
    public static final double ZEROING_SPEED = 0.1;

    public static final double ENCODER_POSITION = 0;

    public static final int INTAKE_CURRENT_LIMIT = 40;
    public static final int PIVOT_CURRENT_LIMIT = 40;
  }
}

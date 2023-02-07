package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final double TAU = Math.PI * 2;

  public static final class Misc {
    public static final int GYRO_PORT = 0;
    public static final int CANDLE_PORT = 0;
  }

  public static final class Joystick {
    public static final int LEFT_JOYSTICK_PORT = 1;
    public static final int RIGHT_JOYSTICK_PORT = 0;

    public static final double MOVE_DEAD_ZONE = 0.15;
    public static final double TURN_DEAD_ZONE = 0.1;

    public static final double ANGLE_DEAD_ZONE = Constants.TAU / 12;

    public static final double MOVE_SENSITIVITY = 1.5;
    public static final double TURN_SENSITIVITY = 1;
  }
  
  public static final class Swerve {
    public static final double TURN_GEAR_RATIO = 150. / 7.;
    public static final double MOVE_GEAR_RATIO = 8.14;

    //meters
    public static final double WIDTH = .548;
    public static final double LENGTH = .548;
    public static final double DIAGONAL = Math.sqrt(WIDTH*WIDTH + LENGTH*LENGTH)/2;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WIDTH/2, LENGTH/2),
      new Translation2d(-WIDTH/2, LENGTH/2),
      new Translation2d(-WIDTH/2, -LENGTH/2),
      new Translation2d(WIDTH/2, -LENGTH/2)
    );

    //School applicable wheel radius
    public static final double WHEELRADIUS = Units.inchesToMeters(1.87);

    //WPI applicable wheel radius
    // public static final double WHEELRADIUS = Units.inchesToMeters(1.925);
    
    //Maximum velocity in NU/100ms
    public static final double MAX_NATIVE_VELOCITY = 22_000;

    public static final int FRONT_RIGHT_TURN_PORT = 0;
    public static final int FRONT_LEFT_TURN_PORT = 1;
    public static final int BACK_LEFT_TURN_PORT = 2;
    public static final int BACK_RIGHT_TURN_PORT = 3;

    public static final int FRONT_RIGHT_MOVE_PORT = 4;
    public static final int FRONT_LEFT_MOVE_PORT = 5;
    public static final int BACK_LEFT_MOVE_PORT = 6;
    public static final int BACK_RIGHT_MOVE_PORT = 7;

    public static final int FRONT_RIGHT_SENSOR_PORT = 0;
    public static final int FRONT_LEFT_SENSOR_PORT = 1;
    public static final int BACK_LEFT_SENSOR_PORT = 2;
    public static final int BACK_RIGHT_SENSOR_PORT = 3;
    
    public static final double FRONT_RIGHT_OFFSET_DEGREES = -163.652;
    public static final double FRONT_LEFT_OFFSET_DEGREES = 160.652;
    public static final double BACK_LEFT_OFFSET_DEGREES = -41.869;
    public static final double BACK_RIGHT_OFFSET_DEGREES = -339.633;

    public static final double FRONT_RIGHT_OFFSET_RADIANS = FRONT_RIGHT_OFFSET_DEGREES * Math.PI/180;
    public static final double FRONT_LEFT_OFFSET_RADIANS = FRONT_LEFT_OFFSET_DEGREES * Math.PI/180;
    public static final double BACK_LEFT_OFFSET_RADIANS = BACK_LEFT_OFFSET_DEGREES * Math.PI/180;
    public static final double BACK_RIGHT_OFFSET_RADIANS = BACK_RIGHT_OFFSET_DEGREES * Math.PI/180;

    public static final double TURN_KF = 0.0475;
    public static final double TURN_KP = 0.35;
    public static final double TURN_KI = 0;
    public static final double TURN_KD = 0.1;

    public static final double MOVE_KF = 0.04; //0.0475;
    public static final double MOVE_KP = 0.05;
    public static final double MOVE_KI = 0.0;
    public static final double MOVE_KD = 0.02;

    public static final double MOD0_AFF = 0.07;
    public static final double MOD1_AFF = 0.07;
    public static final double MOD2_AFF = 0.07;
    public static final double MOD3_AFF = 0.07;
  
    public static final class Balance{
      public static final double K_P = 0.01;
      public static final double K_I = 0.0;
      public static final double K_D = 0.0;

      public static final double MAX_SPEED_PERCENT = 0.3;
    }

    public static final class Auto {
      
      public static final double MAX_SPEED = 4; // m/s
      public static final double MAX_ACCELERATION = 2; // m/s^2

      public static final double MAX_TURNING_SPEED = 1; // r/s
      public static final double MAX_TURNING_ACCELERATION = 0.5; // r/s^2

      public static final double P_X = 5;
      public static final double D_X = 0;
      public static final double P_Y = 5;
      public static final double D_Y = 0;
      public static final double P_THETA = 5;
      public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_TURNING_SPEED,
      MAX_TURNING_ACCELERATION
      );
    }

    public static final class DriftCorrection {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;

      public static final double MAX_ANGULAR_VELOCITY = 0;
    }
  }

  public static final class Grabber{
    public static final int LEFT_GRABBER_PORT = 0;
    public static final int RIGHT_GRABBER_PORT = 0;

    public static final int WRIST_PORT = 0;
  }

  public static final class Arm {
    public static final double PIVOT_GEARRATIO = 192/1; //192 rotations of the motor = 1 rotation of the sprocket
    public static final double EXTENSION_GEARRATION = 5/1;  //5 rotations of the motor = 1 rotation of the pulley
    public static final double PITCH_DIAMETER = Units.inchesToMeters(1.12);
    // One rotation of pulley = 3.5437

    public static final int PIVOT_PORT_1 = 0;
    public static final int PIVOT_PORT_2 = 0;

    public static final int ARM_PORT = 0;

    public static final int EXTEND_SWITCH_PORT = 0;
    public static final int RETRACT_SWITCH_PORT = 0;

    public static final double ARM_KF = 0;
    public static final double ARM_KP = 0;
    public static final double ARM_KI = 0;
    public static final double ARM_KD = 0;

    public static final double ARM_CRUISE_VELOCITY = 0;
    public static final double ARM_ACCELERATION = 0;

    public static final double PIVOT_KF_1 = 0;
    public static final double PIVOT_KP_1 = 0;
    public static final double PIVOT_KI_1 = 0;
    public static final double PIVOT_KD_1 = 0;

    public static final double PIVOT_KF_2 = 0;
    public static final double PIVOT_KP_2 = 0;
    public static final double PIVOT_KI_2 = 0;
    public static final double PIVOT_KD_2 = 0;
  }

  public static final class Field {
    public static final Rotation2d ANGLE_OF_RESISTANCE = Rotation2d.fromRadians(0);
    public static final double K_CARPET = 0.04; // should not be higher than 0.5

    public enum TargetLevel {
      HIGH,
      MID,
      LOW
    }

    public static final double HIGH_TAPE_CAMERA_HEIGHT = 0;
    public static final double MID_TAPE_CAMERA_HEIGHT = 0;
  }

  public static final class Limelight {
    public static final int REFLECTIVE_TAPE_PIPELINE = 0;
    public static final int APRIL_TAG_PIPELINE = 1;
    public static final int CLASSIFICATION_PIPELINE = 2;
    public static final int DETECTION_PIPELINE = 3;
  }
}
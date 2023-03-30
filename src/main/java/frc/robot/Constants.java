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
    public static final int OPERATOR_PORT = 2;

    public static final double MOVE_DEAD_ZONE = 0.15;
    public static final double TURN_DEAD_ZONE = 0.1;

    public static final double ANGLE_DEAD_ZONE = Constants.TAU / 12;

    public static final double MOVE_SENSITIVITY = 1.5;
    public static final double TURN_SENSITIVITY = 1;

    public static final double MANUAL_EXTEND_DEADZONE = 0.1;
    public static final double MANUAL_PIVOT_DEADZONE = 0.1;

    public static final double MANUAL_EXTEND_OUT_SENSITIVITY = 0.18;
    public static final double MANUAL_EXTEND_IN_SENSITIVITY = 0.09;
    public static final double MANUAL_PIVOT_SENSITIVITY = 0.1;
    public static final double MANUAL_WRIST_SENSITIVITY = 0.5;
  }
  
  public static final class Swerve {
    public static final double TURN_GEAR_RATIO = 150. / 7.;
    public static final double MOVE_GEAR_RATIO = 8.14;

    //2.625
    //meters
    public static final double WIDTH = Units.inchesToMeters(26 - 2.625*2);
    public static final double LENGTH = Units.inchesToMeters(26 - 2.625*2);
    public static final double DIAGONAL = Math.sqrt(WIDTH*WIDTH + LENGTH*LENGTH)/2;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WIDTH/2, -LENGTH/2),
      new Translation2d(WIDTH/2, LENGTH/2),
      new Translation2d(-WIDTH/2, LENGTH/2),
      new Translation2d(-WIDTH/2, -LENGTH/2)
    );

    //School applicable wheel radius
    // public static final double WHEELRADIUS = Units.inchesToMeters(1.888);

    //WPI applicable wheel radius
    public static final double WHEELRADIUS = Units.inchesToMeters(1.925);
    
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
    
    public static final double FRONT_RIGHT_OFFSET_DEGREES = -251.191 - 90;
    public static final double FRONT_LEFT_OFFSET_DEGREES = 161.191 - 90;
    public static final double BACK_LEFT_OFFSET_DEGREES = 15.82 - 90;
    public static final double BACK_RIGHT_OFFSET_DEGREES = -309.199 - 90;

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
      public static final double FAST_K_P = 0.01;
      public static final double FAST_K_I = 0.0;
      public static final double FAST_K_D = 0.0;

      public static final double SLOW_K_P = 0.005;
      public static final double SLOW_K_I = 0.0;
      public static final double SLOW_K_D = 0.0;

      // public static final double MAX_SPEED_PERCENT = 0.3;
    }

    public static final class Auto {
      public static final double MAX_SPEED = 4; // m/s
      public static final double MAX_ACCELERATION = 2; // m/s^2

      public static final double MAX_TURNING_SPEED = 1; // r/s
      public static final double MAX_TURNING_ACCELERATION = 0.5; // r/s^2

      public static final double P_X = 5; //5;
      public static final double D_X = 0; //0.005;
      public static final double P_Y = 5; //5;
      public static final double D_Y = 0; //0.005;
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

  public static final class Grabber {
    public static final int LEFT_GRABBER_PORT = 13;
    public static final int RIGHT_GRABBER_PORT = 12;

    public static final int WRIST_PORT = 11;

    // TODO: FILL IN PID LOOP VALUES, CHANGE SPEED, AND CALCULATE ANGLES
    public static final double ORIENTER_KF = 0.001;
    public static final double ORIENTER_KP = 0.2;
    public static final double ORIENTER_KI = 0.0;
    public static final double ORIENTER_KD = 0.0;

    public static final double MAX_TURN_SPEED = 0.6;

    public static final double INTAKE_ANGLE = 4;
    // public static final double INTAKE_ANGLE = 7.5;
    public static final double DOUBLE_STATION_POS = 14;

    public static final double HIGH_ANGLE = 0;
    public static final double MID_ANGLE = 0;
    public static final double LOW_ANGLE = 0;

    public static final double ERROR_ANGLE = 0;

    public static final double GEAR_RATIO = 48 * 36 / 22.;  //TODO: FIND THIS!!!

    //Scorign
    public static final double PREP_CONE_NU = -8.5;
    public static final double SCORE_CONE_NU = 4;
    public static final double CUBE_NU = -9;

    public static final double CONE_RELEASE_SPEED = 0.1; //Speed of rollers to release cone
    public static final double LOW_CONE_RELEASE_SPEED = 0.7;
    public static final double CUBE_RELEASE_SPEED = 0.4;

    //Intkae
    public static final double CONE_INTAKE_SPEED = -0.9;  //Wheels spin in, so the value is negative
    public static final double CUBE_INTAKE_SPEED = 0.4;   //We explicitly state the positive and negative in the command
    public static final double CONE_HOLD_SPEED = -0.1;
    public static final double CUBE_HOLD_SPEED = 0.05;
  }

  public static final class Arm {
    public static final double PIVOT_GEARRATIO = 4*4*4*72/22; //4^3*72 rotations of the motor = 22 rotation of the sprocket
    public static final double NU_PER_MM = 58.4;
    public static final double MM_PER_NU = 0.0171;

    public static final double l0 = 0.690;  //Initial length of arm in meters

    public static final double ABSOLUTE_ENCODER_OFFSET = -17.490234375;

    public static final int PIVOT_PORT_1 = 13;
    public static final int PIVOT_PORT_2 = 15;

    public static final int ARM_PORT = 10;

    public static final int EXTEND_SWITCH_PORT = 0;
    public static final int RETRACT_SWITCH_PORT = 0;

    public static final double ENCODER_OFFSET = -17.490234375;

    public static final double ARM_KF = 0.047;
    public static final double ARM_KP = 0.2;
    public static final double ARM_KI = 0;
    public static final double ARM_KD = 0;

    public static final double ARM_CRUISE_VELOCITY = 50_000;  //40_000
    public static final double ARM_ACCELERATION = 20_000;

    public static final double PIVOT_CRUISE_VELOCITY = 50_000;
    public static final double PIVOT_ACCELERATION = 45_000;

    public static final double PIVOT_KF_1 = 0.045;
    public static final double PIVOT_KP_1 = 0.3;
    public static final double PIVOT_KI_1 = 0;
    public static final double PIVOT_KD_1 = 0;

    public static final double PIVOT_KF_2 = 0.045;
    public static final double PIVOT_KP_2 = 0.3;
    public static final double PIVOT_KI_2 = 0;
    public static final double PIVOT_KD_2 = 0;

    //Scorign (Radians)
    public static final double HIGH_ANGLE = 62 * TAU/360;
    public static final double MID_ANGLE = 68 * TAU/360;
    public static final double LOW_ANGLE = 104 * TAU/360;

    public static final double HIGH_EXTEND_NU = 47_000; //47_000-2_600
    public static final double MID_EXTEND_NU = 17_500;
    public static final double LOW_EXTEND_NU = 0;

    public static final double AUTO_DUNK_ANGLE = 3 * TAU/360;
    public static final double TELEOP_DUNK_ANGLE = 0 * TAU/360;

    public static final double RETRACT_NU = 13_000; //NU to retract back in after scoring to avoid hitting middle node

    //Intkae
    public static final double INTAKE_ANGLE = 112.4 * Constants.TAU/360; //112.5
    public static final double INTAKE_DEADZONE = 1 * TAU/360;

    public static final double DOUBLE_STATION_ANGLE = 43 * TAU/360;
    public static final double DOUBLE_STATION_EXTEND_NU = 14_000;

    public static final double ERROR_ANGLE = 0;

    public static final int EXTEND_FORWARD_SOFT_LIMIT = 55_000;
    public static final int EXTEND_REVERSE_SOFT_LIMIT = 3_000;

    public static final int PIVOT_FORWARD_SOFT_LIMIT = 140_000;
    public static final int PIVOT_REVERSE_SOFT_LIMIT = -140_000;

    public static final double EXTEND_TARGET_DEADZONE = 500;
    public static final double PIVOT_TARGET_DEADZONE = 0.5 * TAU / 360;
    
    public static final class Cube{
      public static final double HIGH_ANGLE = -59 * TAU/360;
      public static final double MID_ANGLE = -62 * TAU/360;
      public static final double LOW_ANGLE = -105 * TAU/360;

      public static final double INTAKE_ANGLE = -111 * Constants.TAU/360;

      public static final double HIGH_EXTEND_NU = 48_000;
      public static final double MID_EXTEND_NU = 20_000;
      public static final double LOW_EXTEND_NU = 0;
    }
  }

  public static final class Field {
    //school
    // public static final Rotation2d ANGLE_OF_RESISTANCE = Rotation2d.fromRadians(Constants.TAU/2);

    //wpi
    public static final Rotation2d ANGLE_OF_RESISTANCE = Rotation2d.fromRadians(Constants.TAU/4);

    public static final double K_CARPET = 0.0; // should not be higher than 0.5

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
    public static final int CONE_CAM = 7;
  }

  public static final class Logging {
    public static final boolean ARM = false;
    public static final boolean GRABBER = false;
    public static final boolean SWERVE = false;
  }
}
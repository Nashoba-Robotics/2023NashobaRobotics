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
  }

  public static final class Joystick {
    public static final int LEFT_JOYSTICK_PORT = 1;
    public static final int RIGHT_JOYSTICK_PORT = 0;

    public static final double MOVE_DEAD_ZONE = 0.15;
    public static final double TURN_DEAD_ZONE = 0.1;

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
    public static final double WHEELRADIUS = Units.inchesToMeters(1.87);

    public static final double MAX_NATIVE_VELOCITY = 22_000;  //Maximum velocity in NU/100ms

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
    
    public static final double FRONT_RIGHT_OFFSET_DEGREES = -163.652; // -163.74, -163.301  -162.598
    public static final double FRONT_LEFT_OFFSET_DEGREES = 160.652; // -300.234, -300.322  -109.863
    public static final double BACK_LEFT_OFFSET_DEGREES = -41.869; // -219.287, -219.551  -37.705 -49.746
    public static final double BACK_RIGHT_OFFSET_DEGREES = -339.633; // -339.521, -340.752, 341.104, 0-340.576, -341.104  21.904
    //-105.645  -58.535
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

    public static final double MOD0_AFF = 0.07; //0.06;
    public static final double MOD1_AFF = 0.07; //0.063;
    public static final double MOD2_AFF = 0.07; //0.06;
    public static final double MOD3_AFF = 0.07; //0.07;

    public static final class Auto {
      
      public static final double MAX_SPEED = 4; // m/s
      public static final double MAX_ACCELERATION = 2; // m/s^2

      public static final double MAX_TURNING_SPEED = 1;
      public static final double MAX_TURNING_ACCELERATION = 0.5;

      public static final double P_X = 5;
      public static final double D_X = 0.001; //0.00001;
      public static final double P_Y = 5; //3.2;
      public static final double D_Y = 0.001; //0;
      public static final double P_THETA = 5; //0;
      public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_TURNING_SPEED,
      MAX_TURNING_ACCELERATION
      );
    }
  }

  public static final class Grabber{
    public static final int LEFT_GRABBER_PORT = 0;
    public static final int RIGHT_GRABBER_PORT = 0;

    public static final int WRIST_PORT = 0;
  }

  public static final class Arm {
    public static final double GEARRATIO = 0;

    public static final int PIVOT_PORT_1 = 0;
    public static final int PIVOT_PORT_2 = 0;

    public static final int EXTEND_PORT = 0;
  }

  public static final class Field {
    public static final Rotation2d ANGLE_OF_RESISTANCE = Rotation2d.fromRadians(0);
    public static final double K_CARPET = 0.04; // should not be higher than 0.5
  }
}


/*
 * Motors:
 * 2 Neo550 for grabber (Spark max)
 * 1 Neo550 for wrist (Spark max)
 * 1 Falcon for extending arm
 * 2 Falcons for pivoting arm
 * 8 Falcons for swerve
 * 
 * Sensors:
 * 2 Limit switches on extending arm
 * Pigeon
 * 
 * Cameras:
 * Game piece sensing
 * 
 * LEDs:
 * CANdle
 */
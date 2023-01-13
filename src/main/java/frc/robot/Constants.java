package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

  public static final class MiscPorts {
    public static final int GYRO_PORT = 0;
  }
  
  public static final class Swerve {
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
    
    public static final double FRONT_RIGHT_OFFSET_DEGREES = -75.63;
    public static final double FRONT_LEFT_OFFSET_DEGREES = -43.066;
    public static final double BACK_LEFT_OFFSET_DEGREES = -121.318;
    public static final double BACK_RIGHT_OFFSET_DEGREES = 96.68;

    public static final double FRONT_RIGHT_OFFSET_RADIANS = 0;
    public static final double FRONT_LEFT_OFFSET_RADIANS = 0;
    public static final double BACK_LEFT_OFFSET_RADIANS = 0;
    public static final double BACK_RIGHT_OFFSET_RADIANS = 0;
  }

  public static class SwerveBase{
    public static final double GEARRATIO = 0;
  }

}

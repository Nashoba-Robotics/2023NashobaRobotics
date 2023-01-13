package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.JoystickValues;
import frc.robot.lib.SwerveState;

public class SwerveDriveSubsystem extends SubsystemBase {

    private SwerveDriveOdometry odometry;
    private SwerveModule[] modules;
    private Pigeon2 gyro;

    private boolean fieldCentric;

    private SwerveDriveSubsystem() {
        gyro = new Pigeon2(Constants.Misc.GYRO_PORT);
        gyro.configFactoryDefault();
        fieldCentric = false;

        modules = new SwerveModule[] {
            new SwerveModule(Constants.Swerve.FRONT_RIGHT_MOVE_PORT, Constants.Swerve.FRONT_RIGHT_TURN_PORT, Constants.Swerve.FRONT_RIGHT_SENSOR_PORT, Constants.Swerve.FRONT_RIGHT_OFFSET_DEGREES),
            new SwerveModule(Constants.Swerve.FRONT_LEFT_MOVE_PORT, Constants.Swerve.FRONT_LEFT_TURN_PORT, Constants.Swerve.FRONT_LEFT_SENSOR_PORT, Constants.Swerve.FRONT_LEFT_OFFSET_DEGREES),
            new SwerveModule(Constants.Swerve.BACK_LEFT_MOVE_PORT, Constants.Swerve.BACK_LEFT_TURN_PORT, Constants.Swerve.BACK_LEFT_SENSOR_PORT, Constants.Swerve.BACK_LEFT_OFFSET_DEGREES),
            new SwerveModule(Constants.Swerve.BACK_RIGHT_MOVE_PORT, Constants.Swerve.BACK_RIGHT_TURN_PORT, Constants.Swerve.BACK_RIGHT_SENSOR_PORT, Constants.Swerve.BACK_RIGHT_OFFSET_DEGREES)
        };

        odometry = new SwerveDriveOdometry(Constants.Swerve.KINEMATICS, Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions());
    }
    
    private static SwerveDriveSubsystem instance;

    public static SwerveDriveSubsystem getInstance() {
        if(instance == null) {
            instance = new SwerveDriveSubsystem();
        }
        return instance;
    }

    private double getGyroAngle() {
        return ((gyro.getYaw() % 360 + 360) % 360 - 180) * Math.PI / 180;
    }

    public void set(JoystickValues joystickValues, double omega) {
        set(joystickValues.x, joystickValues.y, omega);
    }

    public void set(double x, double y, double omega) {        
        if(fieldCentric) {
            double a = Math.atan2(y, x) - getGyroAngle(); //difference between input angle and gyro angle gives desired field relative angle
            double r = Math.sqrt(x*x + y*y); //magnitude of translation vector
            x = r * Math.cos(a);
            y = r * Math.sin(a);
        }
        
        //Repeated equations
        double a = omega * Constants.Swerve.WIDTH/2;
        double b = omega * Constants.Swerve.LENGTH/2;

        //The addition of the movement and rotational vector
        // TODO: Use 2-size arrays or Translation2D instead of independent doubles
        double x0 = x - b;
        double y0 = y + a;

        double x1 = x - b;
        double y1 = y - a;

        double x2 = x + b;
        double y2 = y - a;

        double x3 = x + b;
        double y3 = y + a;

        //Convert to polar
        SwerveState s0 = new SwerveState(
            Math.sqrt(x0*x0 + y0*y0),
            Math.atan2(y0, x0)
            );

        SwerveState s1 = new SwerveState(
            Math.sqrt(x1*x1 + y1*y1),
            Math.atan2(y1, x1)
        );

        SwerveState s2 = new SwerveState(
            Math.sqrt(x2*x2 + y2*y2),
            Math.atan2(y2, x2)
        );

        SwerveState s3 = new SwerveState(
            Math.sqrt(x3*x3 + y3*y3),
            Math.atan2(y3, x3)
        );

        modules[0].set(s0.move, s0.turn);
        modules[1].set(s1.move, s1.turn);
        modules[2].set(s2.move, s2.turn);
        modules[3].set(s3.move, s3.turn);
    }

    public void setDirectly(double speed, double angle) {
        modules[0].set(speed, angle);
        modules[1].set(speed, angle);
        modules[2].set(speed, angle);
        modules[3].set(speed, angle);
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions(), pose);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    //radians
    public void setGyro(double angle) {
        gyro.setYaw(angle * 180 / Math.PI);
    }

    public double[] getModAngles() {
        return new double[] {
            modules[0].getAbsAngle(),
            modules[1].getAbsAngle(),
            modules[2].getAbsAngle(),
            modules[3].getAbsAngle(),
        };
    }

    public double[] getAnglePositions() {
        return new double[] {
            modules[0].getAngle(),
            modules[1].getAngle(),
            modules[2].getAngle(),
            modules[3].getAngle(),
        };
    }

    @Override
    public void periodic(){
        odometry.update(Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions());
    }
}

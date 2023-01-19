package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.SwerveMath;
import frc.robot.lib.util.JoystickValues;
import frc.robot.lib.util.SwerveState;

public class SwerveDriveSubsystem extends SubsystemBase {

    // FORK PULL REQUEST TEST

    private SwerveDriveOdometry odometry;
    private SwerveModule[] modules;
    private Pigeon2 gyro;

    private boolean fieldCentric;

    private SwerveDriveSubsystem() {
        gyro = new Pigeon2(Constants.Misc.GYRO_PORT);
        gyro.configFactoryDefault();
        fieldCentric = true;

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
        return ((gyro.getYaw() % 360 + 360) % 360 - 180) * Constants.TAU / 360;
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
        Translation2d t0 = new Translation2d(x+b, y-a);
        Translation2d t1 = new Translation2d(x+b, y+a);
        Translation2d t2 = new Translation2d(x-b, y+a);
        Translation2d t3 = new Translation2d(x-b, y-a);

        //convert to polar
        SwerveState[] setStates = SwerveState.fromTranslation2d(
            new Translation2d[] {t0, t1, t2, t3}
        );

        setStates = SwerveMath.normalize(setStates);

        set(setStates);
    }

    public void set(SwerveState[] states) {
        for(int i = 0; i < modules.length; i++) {
            modules[i].set(states[i]);
        }
    }

    public void setDirectly(double speed, double angle) {
        modules[0].set(speed, angle);
        modules[1].set(speed, angle);
        modules[2].set(speed, angle);
        modules[3].set(speed, angle);
    }

    private boolean reseting = false;

    public void resetOdometry(Pose2d pose) {
        reseting = true;
        odometry.resetPosition(Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions(), pose);
        reseting = false;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public void setStates(SwerveModuleState[] states) {
        for(int i = 0; i < modules.length; i++) {
            modules[i].set(states[i]);
        }
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    //radians
    public void setGyro(double angle) {
        gyro.setYaw(angle * 180 / Math.PI);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
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
        if(!reseting) odometry.update(Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions());
        Pose2d pose = odometry.getPoseMeters();

        SmartDashboard.putNumber("x", pose.getX());
        SmartDashboard.putNumber("y", pose.getY());
        SmartDashboard.putNumber("angle", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("gyro angle", getGyroAngle());

        for(int i = 0; i < modules.length; i++) {
            SmartDashboard.putNumber("Mod " + i, modules[i].getMovePosition());
        }
    }
}

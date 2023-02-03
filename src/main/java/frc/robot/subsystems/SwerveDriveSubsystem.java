package frc.robot.subsystems;

import java.text.NumberFormat.Style;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LogManager;
import frc.robot.lib.math.SwerveMath;
import frc.robot.lib.math.Units;
import frc.robot.lib.util.CarpetOdometry;
import frc.robot.lib.util.JoystickValues;
import frc.robot.lib.util.SwerveState;

public class SwerveDriveSubsystem extends SubsystemBase {
    private CarpetOdometry odometry;
    private SwerveModule[] modules;
    private Pigeon2 gyro;

    private boolean fieldCentric;

    private PIDController balanceController;
    private PIDController driftController;

    private SwerveDriveSubsystem() {
        gyro = new Pigeon2(Constants.Misc.GYRO_PORT);
        gyro.configFactoryDefault();
        fieldCentric = true;

        modules = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.FRONT_RIGHT_MOVE_PORT, Constants.Swerve.FRONT_RIGHT_TURN_PORT, Constants.Swerve.FRONT_RIGHT_SENSOR_PORT, Constants.Swerve.FRONT_RIGHT_OFFSET_DEGREES, Constants.Swerve.MOD0_AFF),
            new SwerveModule(1, Constants.Swerve.FRONT_LEFT_MOVE_PORT, Constants.Swerve.FRONT_LEFT_TURN_PORT, Constants.Swerve.FRONT_LEFT_SENSOR_PORT, Constants.Swerve.FRONT_LEFT_OFFSET_DEGREES, Constants.Swerve.MOD1_AFF),
            new SwerveModule(2, Constants.Swerve.BACK_LEFT_MOVE_PORT, Constants.Swerve.BACK_LEFT_TURN_PORT, Constants.Swerve.BACK_LEFT_SENSOR_PORT, Constants.Swerve.BACK_LEFT_OFFSET_DEGREES, Constants.Swerve.MOD2_AFF),
            new SwerveModule(3, Constants.Swerve.BACK_RIGHT_MOVE_PORT, Constants.Swerve.BACK_RIGHT_TURN_PORT, Constants.Swerve.BACK_RIGHT_SENSOR_PORT, Constants.Swerve.BACK_RIGHT_OFFSET_DEGREES, Constants.Swerve.MOD3_AFF)
        };

        odometry = new CarpetOdometry(Constants.Swerve.KINEMATICS, Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions(), Constants.Field.ANGLE_OF_RESISTANCE);
    
        balanceController = new PIDController(Constants.Swerve.Balance.K_P, Constants.Swerve.Balance.K_I, Constants.Swerve.Balance.K_D);
        driftController = new PIDController(Constants.Swerve.DriftCorrection.P, Constants.Swerve.DriftCorrection.I, Constants.Swerve.DriftCorrection.D);
    }
    
    private static SwerveDriveSubsystem instance;

    public static SwerveDriveSubsystem getInstance() {
        if(instance == null) {
            instance = new SwerveDriveSubsystem();
        }
        return instance;
    }

    public double getGyroAngle() {
        return ((gyro.getYaw() % 360 + 360) % 360 - 180) * Constants.TAU / 360;
    }

    public double getBalanceAngle() {
        return 0;
    }

    //Convert to radians?
    public double getPitch(){
        return gyro.getPitch();
    }

    public double getRoll(){
        return gyro.getRoll();
    }

    public void set(JoystickValues joystickValues, double omega, boolean driftCorrection) {
        SmartDashboard.putBoolean("running", false);
        if(driftCorrection) {
            if(omega == 0 && (joystickValues.x != 0 || joystickValues.y != 0)) {
                short[] xyz = new short[3];
                gyro.getBiasedAccelerometer(xyz);
                SmartDashboard.putBoolean("running", true);
                omega = driftController.calculate(xyz[0], omega * Constants.Swerve.DriftCorrection.MAX_ANGULAR_VELOCITY);
                SmartDashboard.putNumber("omega", omega);
            }
        }
        set(joystickValues.x, joystickValues.y, omega);
    }

    public void set(double x, double y, double omega) {

        SmartDashboard.putNumber("Set x", x);
        SmartDashboard.putNumber("Set y", y);

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

    private boolean resetting = false;

    public void resetOdometry(Pose2d pose) {
        resetting = true;
        odometry.resetPosition(Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions(), pose);
        resetting = false;
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

    //Set the desired angle for the balancing (level) and the allowed error (deadzone)
    //All in degrees
    public void setDesiredLevel(double angle, double deadzone){
        balanceController.setSetpoint(angle);
        balanceController.setTolerance(deadzone);
    }

    //TODO: Add algorithm to check whether to use Pitch or Roll (Maybe averaging the values?)
    public double getChange(){
        return balanceController.calculate(getRoll());
    }

    @Override
    public void periodic(){
        for(SwerveModule module : modules) module.updateMovePosition();
        if(!resetting) odometry.update(Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions());
        Pose2d pose = odometry.getPoseMeters();

        SmartDashboard.putNumber("x", pose.getX());
        SmartDashboard.putNumber("y", pose.getY());
        SmartDashboard.putNumber("angle", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("gyro angle", getGyroAngle());

        for(SwerveModule module : modules) {
            SmartDashboard.putNumber("Mod " + module.modNumber, module.getMoveVelocity());
            LogManager.appendToLog(Units.NUToMPS(module.getMoveVelocity()), "ActualState:/mod"+module.modNumber);
        }

    }
}

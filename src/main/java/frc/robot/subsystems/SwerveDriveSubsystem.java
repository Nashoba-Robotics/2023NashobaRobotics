package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LogManager;
import frc.robot.Tabs;
import frc.robot.lib.math.SwerveMath;
import frc.robot.lib.math.NRUnits;
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
    private PIDController cardinalController;

    private SwerveDriveSubsystem() {
        gyro = new Pigeon2(Constants.Misc.GYRO_PORT, "drivet");
        gyro.configFactoryDefault();
        gyro.configMountPose(0, 0.308, -0.483);
        fieldCentric = true;

        /*
         * :-0.263672 '
    Pitch :0.307617 '
    Roll  :-0.483398 
         */

        modules = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.FRONT_RIGHT_MOVE_PORT, Constants.Swerve.FRONT_RIGHT_TURN_PORT, Constants.Swerve.FRONT_RIGHT_SENSOR_PORT, Constants.Swerve.FRONT_RIGHT_OFFSET_DEGREES, Constants.Swerve.MOD0_AFF),
            new SwerveModule(1, Constants.Swerve.FRONT_LEFT_MOVE_PORT, Constants.Swerve.FRONT_LEFT_TURN_PORT, Constants.Swerve.FRONT_LEFT_SENSOR_PORT, Constants.Swerve.FRONT_LEFT_OFFSET_DEGREES, Constants.Swerve.MOD1_AFF),
            new SwerveModule(2, Constants.Swerve.BACK_LEFT_MOVE_PORT, Constants.Swerve.BACK_LEFT_TURN_PORT, Constants.Swerve.BACK_LEFT_SENSOR_PORT, Constants.Swerve.BACK_LEFT_OFFSET_DEGREES, Constants.Swerve.MOD2_AFF),
            new SwerveModule(3, Constants.Swerve.BACK_RIGHT_MOVE_PORT, Constants.Swerve.BACK_RIGHT_TURN_PORT, Constants.Swerve.BACK_RIGHT_SENSOR_PORT, Constants.Swerve.BACK_RIGHT_OFFSET_DEGREES, Constants.Swerve.MOD3_AFF)
        };

        odometry = new CarpetOdometry(Constants.Swerve.KINEMATICS, Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions(), Constants.Field.ANGLE_OF_RESISTANCE);
    
        balanceController = new PIDController(Constants.Swerve.Balance.SLOW_K_P, Constants.Swerve.Balance.SLOW_K_I, Constants.Swerve.Balance.SLOW_K_D);
        driftController = new PIDController(Constants.Swerve.DriftCorrection.P, Constants.Swerve.DriftCorrection.I, Constants.Swerve.DriftCorrection.D);
        cardinalController = new PIDController(0.01, 0, 0.0);
        cardinalController.setTolerance(2);

        LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.REFLECTIVE_TAPE_PIPELINE);
    }
    
    private static SwerveDriveSubsystem instance;

    public static SwerveDriveSubsystem getInstance() {
        if(instance == null) {
            instance = new SwerveDriveSubsystem();
        }
        return instance;
    }

    // public void setCardinalTarget(double angle){
    //     cardinalController.setSetpoint(angle);
    // }

    // public double getCardinalGain(){
    //     return cardinalController.calculate(gyro.getYaw());
    // }

    private double setAngle = 0;

    public void turnModulesToAngle(double angle) {
        setAngle = NRUnits.constrainRad(angle - getGyroAngle());
        for(SwerveModule module : modules) {
            module.turn(setAngle);
        }
    }

    public boolean atTurnPos() {
        boolean atPos = true;
        for(SwerveModule module : modules) {
            boolean atAngle = module.atTargetAngle(setAngle)
            || module.atTargetAngle(setAngle + Constants.TAU/2)
            || module.atTargetAngle(setAngle - Constants.TAU/2);
            SmartDashboard.putBoolean("mod"+module.modNumber, atAngle);
            atPos = atPos && atAngle;
        }
        return atPos;
    }

    public void moveYNU(double y) {
        setAngle = NRUnits.constrainRad(Math.atan2(y, 0) - getGyroAngle()); //difference between input angle and gyro angle gives desired field relative angle

        for(SwerveModule module : modules) {
            module.moveNUDeg(y, setAngle);
        }
    }

    public void singleModNUTest(int NU) {
        modules[1].moveNUDeg(NU, 0);
    }

    public double getYaw(){
        return gyro.getYaw();
    }
    public boolean atCardinalAngle(){
        return cardinalController.atSetpoint();
    }

    public void brake() {
        modules[0].set(0, Constants.TAU/8, false);
        modules[1].set(0, -Constants.TAU/8, false);
        modules[2].set(0, Constants.TAU/8, false);
        modules[3].set(0, -Constants.TAU/8, false);
    }

    public double getGyroAngle() {
        return NRUnits.constrainDeg(gyro.getYaw()) * Constants.TAU / 360;
    }

    //Don't know if this will work
    public double getBalanceAngle() {
        double pitch = getPitch();
        double roll = getRoll();
        // What to do if pitch and roll are negative and positive? (I don't think it will happen)
        if(Math.signum(pitch) == -1 && Math.signum(roll) == -1)
        return -Math.sqrt(pitch*pitch + roll*roll);
        return Math.sqrt(pitch*pitch + roll*roll);
    }

    //Convert to radians?
    public double getPitch(){
        return gyro.getPitch();
    }

    public double getRoll(){
        return gyro.getRoll();
    }

    public void set(JoystickValues joystickValues, double omega) {
        set(joystickValues.x, joystickValues.y, omega);
    }

    public void set(double x, double y, double omega) {

        SmartDashboard.putNumber("Set x", x);
        SmartDashboard.putNumber("Set y", y);

        if(fieldCentric) {
            double angleDiff = Math.atan2(y, x) - getGyroAngle(); //difference between input angle and gyro angle gives desired field relative angle
            double r = Math.sqrt(x*x + y*y); //magnitude of translation vector
            x = r * Math.cos(angleDiff);
            y = r * Math.sin(angleDiff);
        }
        
        //Repeated equations
        double a = omega * Constants.Swerve.WIDTH/2;
        double b = omega * Constants.Swerve.LENGTH/2;

        //The addition of the movement and rotational vector
        Translation2d t0 = new Translation2d(x-b, y-a);
        Translation2d t1 = new Translation2d(x+b, y-a);
        Translation2d t2 = new Translation2d(x+b, y+a);
        Translation2d t3 = new Translation2d(x-b, y+a);

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

    public void setAngle(double angle){ //rad
        modules[0].turn(angle);
        modules[1].turn(angle);
        modules[2].turn(angle);
        modules[3].turn(angle);
    }

    public void driveNU(double NU){
        modules[0].setMovePos(NU);
        modules[1].setMovePos(NU);
        modules[2].setMovePos(NU);
        modules[3].setMovePos(NU);
    }

    public double getMod1NU(){
        return modules[0].getNU();
    }

    private boolean resetting = false;

    public void resetOdometry(Pose2d pose) {
        resetting = true;
        odometry.resetPosition(Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions(), pose);
        resetting = false;
    }

    public void resetOdometryOverrideAngle(Pose2d pose, Rotation2d angle) {
        resetting = true;
        odometry.resetPosition(angle, getSwerveModulePositions(), pose);
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
            // SmartDashboard.putNumber("SetAngle"+i, states[i].angle.getDegrees());
            modules[i].set(states[i]);
        }
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public boolean isFieldCentric() {
        return this.fieldCentric;
    }

    //radians
    public void setGyro(double angle) {
        gyro.setYaw(angle * 180 / Math.PI);
    }

    public void zeroYaw() {
        gyro.setYaw(0);
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

    public void setBalancePID(double kP, double kI, double kD){
        balanceController.setP(kP);
        balanceController.setI(kI);
        balanceController.setD(kD);
    }

    public boolean balanced(){
        return balanceController.atSetpoint();
    }

    public boolean isLevel(){
        return Math.abs(getRoll()) < 1.5;
    }

    public boolean notLevel() {
        return !isLevel();
    }

    public boolean reallyNotLevel() {
        return !(Math.abs(getRoll()) < 8);
    }

    public boolean levelNegative() {
        return getRoll() < -1;
    }

    public boolean levelPositive() {
        return getRoll() > 1;
    }

    //TODO: Add algorithm to check whether to use Pitch or Roll (Maybe averaging the values?)
    public double getChange(){
        return balanceController.calculate(getRoll());
        //return balanceController.calculate(getBalanceAngle());
    }

    public void resetModulesAbsolute() {
        for(SwerveModule module : modules) {
            module.resetTurnToAbsolute();
        }
    }

    public double getXVelocity() {
        ChassisSpeeds speed = Constants.Swerve.KINEMATICS.toChassisSpeeds(
            modules[0].getSwerveState(),
            modules[1].getSwerveState(),
            modules[2].getSwerveState(),
            modules[3].getSwerveState());

        return speed.vxMetersPerSecond;
    }

    public double getYVelocity() {
        ChassisSpeeds speed = Constants.Swerve.KINEMATICS.toChassisSpeeds(
            modules[0].getSwerveState(),
            modules[1].getSwerveState(),
            modules[2].getSwerveState(),
            modules[3].getSwerveState());

        return speed.vyMetersPerSecond;
    }

    public double getVelocity() {
        ChassisSpeeds speed = Constants.Swerve.KINEMATICS.toChassisSpeeds(
            modules[0].getSwerveState(),
            modules[1].getSwerveState(),
            modules[2].getSwerveState(),
            modules[3].getSwerveState());

        return Math.abs(Math.sqrt(speed.vxMetersPerSecond*speed.vxMetersPerSecond + speed.vyMetersPerSecond*speed.vyMetersPerSecond));
    }

    @Override
    public void periodic(){
        for(SwerveModule module : modules) module.updateMovePosition();

        if(!resetting) odometry.update(Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions());

        // LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.APRIL_TAG_PIPELINE);
        SmartDashboard.putNumber("Pipeline", LimelightSubsystem.getInstance().getPipeline());

        if(LimelightSubsystem.getInstance().getPipeline() != Constants.Limelight.APRIL_TAG_PIPELINE) LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.APRIL_TAG_PIPELINE);
        // if(LimelightSubsystem.getInstance().isTarget()) {
        //     double limelightWeight = 0.1;
        //     double odometryWeight = 1 - limelightWeight;

        //     Translation2d limelightPose = LimelightSubsystem.getInstance().getRobotPose().getTranslation();
        //         Translation2d odometryPose = SwerveDriveSubsystem.getInstance().getPose().getTranslation();
        //         Pose2d currPose = new Pose2d(limelightPose.getX() * limelightWeight + odometryPose.getX() * odometryWeight, limelightPose.getY() * limelightWeight + odometryPose.getY() * odometryWeight, Rotation2d.fromRadians(SwerveDriveSubsystem.getInstance().getGyroAngle()));
        //         SwerveDriveSubsystem.getInstance().resetOdometry(currPose);
        // }

        SmartDashboard.putNumber("mod0Ang", modules[0].getTurnAngle());
        SmartDashboard.putNumber("mod1Ang", modules[1].getTurnAngle());
        SmartDashboard.putNumber("mod2Ang", modules[2].getTurnAngle());
        SmartDashboard.putNumber("mod3Ang", modules[3].getTurnAngle());
        SmartDashboard.putNumber("angleSetPoint", setAngle);

        SmartDashboard.putNumber("RobotVelocity", getVelocity());
        SmartDashboard.putNumber("XVelocity", getXVelocity());
        SmartDashboard.putNumber("YVelocity", getYVelocity());

        Pose2d pose = odometry.getPoseMeters();

        SmartDashboard.putNumber("x", pose.getX());
        SmartDashboard.putNumber("y", pose.getY());
        SmartDashboard.putNumber("angle", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("gyro angle", getGyroAngle());
        Tabs.Comp.displayGyro(getGyroAngle());

        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Roll", getRoll());
        SmartDashboard.putNumber("Change", getChange());
        SmartDashboard.putBoolean("At Setpoint",balanced());

        if(Constants.Logging.SWERVE) {
            for(SwerveModule module : modules) {
                SmartDashboard.putNumber("Mod " + module.modNumber, module.getMoveVelocity());
                LogManager.appendToLog(module.getMoveVelocity(), "Swerve/:Mod"+module.modNumber+"/Velocity");
                LogManager.appendToLog(module.getAngle(), "Swerve/:Mod"+module.modNumber+"/Angle");
            }

            LogManager.appendToLog(gyro.getYaw(), "Gyro:/Yaw");
            LogManager.appendToLog(gyro.getPitch(), "Gyro:/Pitch");
            LogManager.appendToLog(gyro.getRoll(), "Gyro:/Roll");

            LogManager.appendToLog(odometry.getPoseMeters().getX(), "Swerve:/Odometry/X");
            LogManager.appendToLog(odometry.getPoseMeters().getY(), "Swerve:/Odometry/Y");
            LogManager.appendToLog(odometry.getPoseMeters().getRotation().getDegrees(), "Swerve:/Odometry/Angle");
        }
    }
}

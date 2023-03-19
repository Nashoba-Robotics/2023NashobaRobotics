package frc.robot.commands.score;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.math.NRUnits;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PrepHighConeCommand extends CommandBase{
    double l0 = 0.690;
    double targetPos;
    double setPos2;
    double setPos3;
    double multiplier;

    double lastPos;
    double lastPos2;
    boolean joystick0;
    boolean joystick02;
    boolean gotToStart;
    boolean atSetPoint2;

    public PrepHighConeCommand(){
        addRequirements(ArmSubsystem.getInstance());
    }

    public double NUtoMPS(double NU){
        //Converts NU/100ms to meters/s
        NU *= 10;
        NU /= 58.4;
        NU /= 1000;

        return NU;
    }

    public double mpsToNU(double mps){
        mps *= 1000;
        mps *= 58.4;
        mps /= 10;

        return mps;
    }

    private double NUToRadiansPerSecond(double NU) {
        NU = NRUnits.Pivot.NUToRad(NU);
        return NU * 10;
    }

    @Override
    public void initialize() {
        lastPos = 0;
        lastPos2 = 0;
        joystick0 = false;
        joystick02 = false;
        gotToStart = false;
        atSetPoint2 = false;

        double gyroAngle = SwerveDriveSubsystem.getInstance().getGyroAngle();

        boolean scoreFront = gyroAngle > Constants.TAU/4 || gyroAngle < -Constants.TAU/4;

        multiplier = scoreFront ? 1 : -1;

        double prepAngle = 0;
        if(multiplier == 1) prepAngle = Constants.Arm.HIGH_ANGLE + (1)*Constants.TAU/360;
        else prepAngle = -(Constants.Arm.HIGH_ANGLE - 0.1*Constants.TAU/360);

        ArmSubsystem.getInstance().pivot(prepAngle);
        ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU);
        GrabberSubsystem.getInstance().orientPos(-8.5 * multiplier);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(50_000);
        ArmSubsystem.getInstance().setPivotAcceleration(55_000);

        ArmSubsystem.getInstance().setExtendAcceleration(50_000);

        targetPos = Constants.Arm.HIGH_EXTEND_NU;
        setPos2 = prepAngle;
        setPos3 = Constants.Grabber.SCORE_NU * multiplier;
    }

    @Override
    public void execute() {
        //l0 = 0.690m
        double pivotAngle = ArmSubsystem.getInstance().getAngle();
        double pivotSpeed = ArmSubsystem.getInstance().getPivotSpeed();
        pivotSpeed = NUToRadiansPerSecond(pivotSpeed);

        double extendSpeed = l0 * Math.tan(pivotAngle)/Math.cos(pivotAngle)*pivotSpeed;
        extendSpeed = mpsToNU(extendSpeed);

        ArmSubsystem.getInstance().setExtendCruiseVelocity(extendSpeed);

        if(Math.abs(ArmSubsystem.getInstance().getExtendNU() - Constants.Arm.HIGH_EXTEND_NU) < 1000){
            GrabberSubsystem.getInstance().orientPos(Constants.Grabber.SCORE_NU * multiplier);
        }

        SmartDashboard.putNumber("Actual NU", ArmSubsystem.getInstance().getExtendNU());
        SmartDashboard.putNumber("Pivot Speed", ArmSubsystem.getInstance().getExtendNUSpeed());


        //MANUAL!!!!!!!!!!!!!!!!!

        if(!gotToStart && Math.abs(ArmSubsystem.getInstance().getPos() - targetPos) < 500) gotToStart = true;
        if(gotToStart) {
            double y = RobotContainer.operatorController.getThrottle() ;
            y = Math.abs(y) < 0.1 ? 0 : (y-0.1)/0.9;    //Put deadzone in Constants
            if(y < 0) y *= 0.6;
            else y *= 0.3;
            if(y == 0){ // If there isn't any input, maintain the position
                if(!joystick0){
                    joystick0 = true;
                    lastPos = ArmSubsystem.getInstance().getPos();
                }
                ArmSubsystem.getInstance().extendNU(lastPos);
            }
            else{
                ArmSubsystem.getInstance().set(-y*0.3);
                joystick0 = false;
            }

            //Added pivoting manual
            if(Math.abs(ArmSubsystem.getInstance().getAngle() - setPos2) < 0.5 * Constants.TAU / 360){
                atSetPoint2 = true;
            } 
    
            if(atSetPoint2) {
                double pivotX = RobotContainer.operatorController.getX() * multiplier;
                pivotX = Math.abs(pivotX) < 0.1 ? 0 : (pivotX-0.1)/0.9;
                if(pivotX == 0){ // If there isn't any input, maintain the position
                    if(!joystick02){
                        joystick02 = true;
                        lastPos2 = ArmSubsystem.getInstance().getAngle();
                    }
                    ArmSubsystem.getInstance().pivot(lastPos2);
                    SmartDashboard.putNumber("lastPos2", lastPos2);
                }
                else{
                    ArmSubsystem.getInstance().setPivot(pivotX*0.1);
                    joystick02 = false;
                }
                SmartDashboard.putBoolean("Manual", joystick02);
                SmartDashboard.putNumber("SetPoint", lastPos2);
            }

            if(RobotContainer.operatorController.pov(0).getAsBoolean()) setPos3 -= 0.5 * multiplier;
            if(RobotContainer.operatorController.pov(180).getAsBoolean()) setPos3 += 0.5 * multiplier;

            GrabberSubsystem.getInstance().orientPos(setPos3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        ArmSubsystem.getInstance().setDefaultAcceleration();

        ArmSubsystem.getInstance().setExtendCruiseVelocity(Constants.Arm.ARM_CRUISE_VELOCITY);
        ArmSubsystem.getInstance().setExtendAcceleration(15_000);

        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(0);
        GrabberSubsystem.getInstance().orientPos(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands.score;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.Field.TargetLevel;


public class AutoDirectionalPrepHeightCommand extends CommandBase {
    TargetLevel targetLevel;
    double targetPos;
    boolean autoDir = true;

    double lastPos;
    double lastPos2;
    boolean joystick0;
    boolean joystick02;
    boolean gotToStart;
    boolean atSetPoint2;
    double setPos2;
    double setPos3;

    boolean scoreFront;
    int multiplier;

    public AutoDirectionalPrepHeightCommand(TargetLevel targetLevel) {
        this.targetLevel = targetLevel;
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    public AutoDirectionalPrepHeightCommand(TargetLevel targetLevel, boolean autoDir){
        this.autoDir = autoDir;
        this.targetLevel = targetLevel;
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    public void initialize() {
        lastPos = 0;
        lastPos2 = 0;
        joystick0 = false;
        joystick02 = false;
        gotToStart = false;
        atSetPoint2 = false;
        setPos2 = 0;
        setPos3 = 0;

        ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        ArmSubsystem.getInstance().setDefaultAcceleration();

        //Assuming that we are oriented correctly to score the cone, the way the arm goes down will change
        //Really just making all of the normal values negative of what they are in order to reflect them vertically
        double gyroAngle = SwerveDriveSubsystem.getInstance().getGyroAngle();
        scoreFront = gyroAngle > Constants.TAU/4 || gyroAngle < -Constants.TAU/4;
        multiplier = scoreFront ? 1 : -1;

        switch(targetLevel) {
            case HIGH: 
            if(!autoDir) multiplier = 1;
            GrabberSubsystem.getInstance().orientPos(Constants.Grabber.SCORE_NU*multiplier);
            if(multiplier == 1) ArmSubsystem.getInstance().pivot(Constants.Arm.HIGH_ANGLE);
            if(multiplier == -1) ArmSubsystem.getInstance().pivot(-(Constants.Arm.HIGH_ANGLE));
             if(DriverStation.isAutonomous()){
                ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU-2100);
             }
             else ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU);
             targetPos = Constants.Arm.HIGH_EXTEND_NU;
             setPos2 = Constants.Arm.HIGH_ANGLE * multiplier;
             setPos3 = Constants.Grabber.SCORE_NU * multiplier;
             break;
            case MID: 
             GrabberSubsystem.getInstance().orientPos(Constants.Grabber.SCORE_NU * multiplier);
             ArmSubsystem.getInstance().pivot(Constants.Arm.MID_ANGLE * multiplier);
             ArmSubsystem.getInstance().extendNU(Constants.Arm.MID_EXTEND_NU);
             targetPos = Constants.Arm.MID_EXTEND_NU;
             setPos2 = Constants.Arm.MID_ANGLE * multiplier;
             setPos3 = Constants.Grabber.SCORE_NU * multiplier;
             break;
           case LOW: 
            GrabberSubsystem.getInstance().orientPos(Constants.Grabber.SCORE_NU * multiplier);
            ArmSubsystem.getInstance().pivot(Constants.Arm.LOW_ANGLE * multiplier);
            ArmSubsystem.getInstance().extendNU(Constants.Arm.LOW_EXTEND_NU);
            targetPos = Constants.Arm.LOW_EXTEND_NU;
            setPos2 = Constants.Arm.LOW_ANGLE * multiplier;
            setPos3 = Constants.Grabber.SCORE_NU * multiplier;
            break;
        }
        gotToStart = false;
    }

    @Override
    public void execute() {
        //Manual Extension:
        //Make sure that the arm has reached its desired extend position before we allow manual movement to happen
        if(!gotToStart && Math.abs(ArmSubsystem.getInstance().getPos() - targetPos) < 500) gotToStart = true;
        if(gotToStart) {
            double y = RobotContainer.operatorController.getThrottle() ;
            y = Math.abs(y) < 0.1 ? 0 : (y-0.1)/0.9;    //Put deadzone in Constants
            if(y < 0) y *= 0.6;
            else y *= 0.3;
            if(y == 0){ // If there isn't any input, maintain the position
                if(!joystick0){ //When the joystick is first zeroed
                    joystick0 = true;   //Flag variable to see if the joystick was zeroed
                    lastPos = ArmSubsystem.getInstance().getPos();
                    // ^ Without this, the arm goes in bit by bit because gravity, lastPos reads that position and tells the extension to go there
                }
                ArmSubsystem.getInstance().extendNU(lastPos);
            }
            else{
                ArmSubsystem.getInstance().set(-y*0.3);
                joystick0 = false;
            }

            //Manual Pivot:
            //All of the logic is the same compared to the extension
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

            //Manual Wrist:
            if(RobotContainer.operatorController.pov(0).getAsBoolean()) setPos3 -= 0.15 * multiplier;
            if(RobotContainer.operatorController.pov(180).getAsBoolean()) setPos3 += 0.15 * multiplier;

            GrabberSubsystem.getInstance().orientPos(setPos3);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}

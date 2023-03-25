package frc.robot.commands.score;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.Field.TargetLevel;


public class CubeAutoDirectionalPrepHeightCommand extends CommandBase {
    TargetLevel targetLevel;
    double targetPos;
    
    double lastPos;
    double lastPos2;
    boolean joystick0;
    boolean joystick02;
    boolean gotToStart;
    boolean atSetPoint2;
<<<<<<< HEAD
    double setPos2;
    double setPos3;
    boolean resetEncoder;
=======
    double targetPivot;
    double targetWrist;
>>>>>>> 90730b1855bf25fcd8a23ea0a3e95686564be369

    boolean scoreFront;
    int multiplier;

    public CubeAutoDirectionalPrepHeightCommand(TargetLevel targetLevel) {
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
        targetPivot = 0;
        targetWrist = 0;

        ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        ArmSubsystem.getInstance().setDefaultAcceleration();
        double gyroAngle = SwerveDriveSubsystem.getInstance().getGyroAngle();
        scoreFront = gyroAngle > Constants.TAU/4 || gyroAngle < -Constants.TAU/4;

        multiplier = scoreFront ? -1 : 1;

        switch(targetLevel) {
            case HIGH: 
            //  GrabberSubsystem.getInstance().orientPos(1);
             ArmSubsystem.getInstance().pivot(Constants.Arm.Cube.HIGH_ANGLE * multiplier);
             ArmSubsystem.getInstance().extendNU(Constants.Arm.Cube.HIGH_EXTEND_NU);
             targetPos = Constants.Arm.Cube.HIGH_EXTEND_NU;
             targetPivot = Constants.Arm.Cube.HIGH_ANGLE * multiplier;
             targetWrist = GrabberSubsystem.getInstance().getOrientPos();
             break;
            case MID: 
            //  GrabberSubsystem.getInstance().orientPos(Constants.Grabber.SCORE_NU * multiplier);
             ArmSubsystem.getInstance().pivot(Constants.Arm.Cube.MID_ANGLE * multiplier);
             ArmSubsystem.getInstance().extendNU(Constants.Arm.Cube.MID_EXTEND_NU);
             targetPos = Constants.Arm.Cube.MID_EXTEND_NU;
             targetPivot = Constants.Arm.Cube.MID_ANGLE * multiplier;
             targetWrist = GrabberSubsystem.getInstance().getOrientPos();
             break;
           case LOW: 
            // GrabberSubsystem.getInstance().orientPos(Constants.Grabber.SCORE_NU * multiplier);
            ArmSubsystem.getInstance().setPivotCruiseVelocity(60_000);
            ArmSubsystem.getInstance().setPivotAcceleration(60_000);
            ArmSubsystem.getInstance().pivot(Constants.Arm.Cube.LOW_ANGLE * multiplier);
            ArmSubsystem.getInstance().extendNU(Constants.Arm.Cube.LOW_EXTEND_NU);
            targetPos = Constants.Arm.Cube.LOW_EXTEND_NU;
            targetPivot = Constants.Arm.Cube.LOW_ANGLE * multiplier;
            targetWrist = GrabberSubsystem.getInstance().getOrientPos();
            break;
        }
        gotToStart = false;
        resetEncoder = false;
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("manual", gotToStart);
        SmartDashboard.putNumber("Arm nu", ArmSubsystem.getInstance().getPos());
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
            if(Math.abs(ArmSubsystem.getInstance().getAngle() - targetPivot) < 0.5 * Constants.TAU / 360){
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

            if(RobotContainer.operatorController.pov(0).getAsBoolean()) targetWrist -= 0.15 * multiplier;
            if(RobotContainer.operatorController.pov(180).getAsBoolean()) targetWrist += 0.15 * multiplier;

            GrabberSubsystem.getInstance().orientPos(setPos3);

            if(!resetEncoder && Math.abs(ArmSubsystem.getInstance().getAngle()-setPos2) <= Constants.Arm.INTAKE_DEADZONE){
                ArmSubsystem.getInstance().resetPivotNU();
                resetEncoder = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // GrabberSubsystem.getInstance().orientPos(3 * multiplier);
    }

    public boolean isFinished() {
        return false;
    }
}

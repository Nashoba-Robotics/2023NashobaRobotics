package frc.robot.commands.score;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tabs;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsytem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.Field.TargetLevel;


public class CubeAutoDirectionalPrepHeightCommand extends CommandBase {
    TargetLevel targetLevel;
    boolean scoreFront;
    int multiplier;

    boolean resetEncoder;

    double targetPos;
    double lastPos;
    boolean extensionMan0;
    boolean gotToStart;

    double lastPivot;
    double targetPivot;
    boolean pivotMan0;
    boolean atPivot;
    
    boolean atStartDeg;
    double targetWrist;

    public CubeAutoDirectionalPrepHeightCommand(TargetLevel targetLevel) {
        this.targetLevel = targetLevel;
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    public void initialize() {
        lastPos = 0;
        lastPivot = 0;
        extensionMan0 = false;
        pivotMan0 = false;
        gotToStart = false;
        atPivot = false;
        targetPivot = 0;
        targetWrist = 0;
        resetEncoder = false;

        ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        ArmSubsystem.getInstance().setDefaultAcceleration();
        double gyroAngle = SwerveDriveSubsystem.getInstance().getGyroAngle();
        scoreFront = gyroAngle > Constants.TAU/4 || gyroAngle < -Constants.TAU/4;

        multiplier = scoreFront ? -1 : 1;

        switch(targetLevel) {
            case HIGH:  //<-- Not used
             ArmSubsystem.getInstance().pivot(Constants.Arm.Cube.HIGH_ANGLE * multiplier);
             ArmSubsystem.getInstance().extendNU(Constants.Arm.Cube.HIGH_EXTEND_NU);
             targetPos = Constants.Arm.Cube.HIGH_EXTEND_NU;
             targetPivot = Constants.Arm.Cube.HIGH_ANGLE * multiplier;
             targetWrist = Constants.Grabber.CUBE_NU;
             break;
            case MID: 
             ArmSubsystem.getInstance().pivot(Constants.Arm.Cube.MID_ANGLE * multiplier);
             ArmSubsystem.getInstance().extendNU(Constants.Arm.Cube.MID_EXTEND_NU);
             targetPos = Constants.Arm.Cube.MID_EXTEND_NU;
             targetPivot = Constants.Arm.Cube.MID_ANGLE * multiplier;
             targetWrist = Constants.Grabber.CUBE_NU;
             break;
           case LOW: 
            ArmSubsystem.getInstance().setPivotCruiseVelocity(60_000);
            ArmSubsystem.getInstance().setPivotAcceleration(60_000);
            ArmSubsystem.getInstance().pivot(Constants.Arm.Cube.LOW_ANGLE * multiplier);
            ArmSubsystem.getInstance().extendNU(Constants.Arm.Cube.LOW_EXTEND_NU);
            targetPos = Constants.Arm.Cube.LOW_EXTEND_NU;
            targetPivot = Constants.Arm.Cube.LOW_ANGLE * multiplier;
            targetWrist = Constants.Grabber.CUBE_NU;
            break;
        }
        gotToStart = false;
        resetEncoder = false;

        atStartDeg = Math.abs(ArmSubsystem.getInstance().getAngle() - 22*Constants.TAU/360) < 1*Constants.TAU/360;

        Tabs.Comp.setExtendTarget(targetPos);
        Tabs.Comp.setPivotTarget(targetPivot);
        Tabs.Comp.setWristTarget(Constants.Grabber.CUBE_NU);
    }

    @Override
    public void execute() {
        if(!DriverStation.isAutonomous() && !atStartDeg && Math.abs(targetPivot) < Constants.TAU/4){
            ArmSubsystem.getInstance().pivot(-22*Constants.TAU/360 * multiplier);
            ArmSubsystem.getInstance().extendNU(3_000);
            if(Math.abs(Math.abs(ArmSubsystem.getInstance().getAngle()) - 22*Constants.TAU/360) < 1*Constants.TAU/360){
                ArmSubsystem.getInstance().pivot(targetPivot);
                ArmSubsystem.getInstance().extendNU(targetPos);
                atStartDeg = true;
            }
        }
        if(!gotToStart && Math.abs(ArmSubsystem.getInstance().getPos() - targetPos) < Constants.Arm.EXTEND_TARGET_DEADZONE) gotToStart = true;
        if(gotToStart) {
            double y = JoystickSubsytem.getInstance().getManualExtend();
            if(y < 0) y *= Constants.Joystick.MANUAL_EXTEND_OUT_SENSITIVITY; //Negative is extend out
            else if(y > 0) y *= Constants.Joystick.MANUAL_EXTEND_IN_SENSITIVITY;    //Positive is retract in

            if(y == 0){ // If there isn't any input, maintain the position
                if(!extensionMan0){
                    extensionMan0 = true;
                    lastPos = ArmSubsystem.getInstance().getPos();
                }
                ArmSubsystem.getInstance().extendNU(lastPos);
            }
            else{
                ArmSubsystem.getInstance().set(-y);
                extensionMan0 = false;
            }

            //Added pivoting manual
            if(Math.abs(ArmSubsystem.getInstance().getAngle() - targetPivot) < Constants.Arm.PIVOT_TARGET_DEADZONE){
                atPivot = true;
            } 
            if(atPivot) {
                double pivotX = JoystickSubsytem.getInstance().getManualPivot();
                if(pivotX == 0){ // If there isn't any input, maintain the position
                    if(!pivotMan0){
                        pivotMan0 = true;
                        lastPivot = ArmSubsystem.getInstance().getAngle();
                    }
                    ArmSubsystem.getInstance().pivot(lastPivot);
                }
                else{
                    ArmSubsystem.getInstance().setPivot(pivotX);
                    pivotMan0 = false;
                }
            }

            if(RobotContainer.operatorController.pov(0).getAsBoolean()) targetWrist -= Constants.Joystick.MANUAL_WRIST_SENSITIVITY * multiplier;
            if(RobotContainer.operatorController.pov(180).getAsBoolean()) targetWrist += Constants.Joystick.MANUAL_WRIST_SENSITIVITY * multiplier;

            GrabberSubsystem.getInstance().orientPos(targetWrist);

            if(!resetEncoder && 
            Math.abs(ArmSubsystem.getInstance().getAngle()-targetPivot) <= 1 * Constants.TAU/360 && 
            Math.abs(ArmSubsystem.getInstance().getPivotSpeed()) < 10){
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

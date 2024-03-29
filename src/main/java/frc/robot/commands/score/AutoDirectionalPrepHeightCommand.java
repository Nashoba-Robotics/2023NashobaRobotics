package frc.robot.commands.score;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Tabs;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.Robot.RobotState;


public class AutoDirectionalPrepHeightCommand extends CommandBase {
    TargetLevel targetLevel;
    int multiplier;
    boolean autoDir = true;
    boolean scoreFront;
    boolean gotToStart;

    double targetPos;
    double lastPos;
    boolean extensionMan0;
    
    double targetPivot;
    double lastPivot;
    boolean atPivot;
    boolean pivotMan0;

    double targetWrist;

    boolean atStartDeg;
    boolean wait;

    boolean resetEncoder;    

    public AutoDirectionalPrepHeightCommand(TargetLevel targetLevel) {
        this.targetLevel = targetLevel;
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    //When don't we want to use auto directional? IN AUTO!! to make sure that at least the first game piece is scored
    public AutoDirectionalPrepHeightCommand(TargetLevel targetLevel, boolean autoDir){
        this.autoDir = autoDir;
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
        resetEncoder = false;
        targetPivot = 0;
        targetWrist = 0;

        if(DriverStation.isAutonomous()){
            ArmSubsystem.getInstance().setAutoSpeeds();
        }
        else {
            ArmSubsystem.getInstance().setDefaultCruiseVelocity();
            ArmSubsystem.getInstance().setDefaultAcceleration();
        }

        //Assuming that we are oriented correctly to score the cone, the way the arm goes down will change
        //Really just making all of the normal pivot values negative of what they are in order to reflect them vertically
        double gyroAngle = SwerveDriveSubsystem.getInstance().getGyroAngle();
        scoreFront = gyroAngle > Constants.TAU/4 || gyroAngle < -Constants.TAU/4;
        multiplier = scoreFront ? 1 : -1;

        switch(targetLevel) {
            case HIGH:
            if(!autoDir) multiplier = 1;
            if(multiplier == 1){
                targetPivot = Constants.Arm.HIGH_FRONT_ANGLE;
                targetWrist = Constants.Grabber.PREP_CONE_FRONT_NU;
            } 
            if(multiplier == -1){
                targetPivot = Constants.Arm.HIGH_BACK_ANGLE;
                targetWrist = Constants.Grabber.PREP_CONE_BACK_NU;
            } 
             if(DriverStation.isAutonomous()){
                targetPos = Constants.Arm.HIGH_EXTEND_NU-1300/2048.;  //61000+1500
             }
             else targetPos = Constants.Arm.HIGH_EXTEND_NU;

             ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU);
             ArmSubsystem.getInstance().pivot(targetPivot);
             GrabberSubsystem.getInstance().orientPos(targetWrist);
             break;
            case MID: 
                GrabberSubsystem.getInstance().orientPos(Constants.Grabber.PREP_CONE_FRONT_NU * multiplier);
                ArmSubsystem.getInstance().pivot(Constants.Arm.MID_ANGLE * multiplier);
                ArmSubsystem.getInstance().extendNU(Constants.Arm.MID_EXTEND_NU);
                targetPos = Constants.Arm.MID_EXTEND_NU;
                targetPivot = Constants.Arm.MID_ANGLE * multiplier;
                targetWrist = Constants.Grabber.PREP_CONE_FRONT_NU * multiplier;
                break;
           case LOW: 
                ArmSubsystem.getInstance().setPivotCruiseVelocity(100);
                ArmSubsystem.getInstance().setPivotAcceleration(293);   //No way in hell that this is right. Why did we make low different from everything else anyways?
                GrabberSubsystem.getInstance().orientPos(Constants.Grabber.PREP_CONE_FRONT_NU * multiplier);
                ArmSubsystem.getInstance().pivot(Constants.Arm.LOW_ANGLE * multiplier);
                ArmSubsystem.getInstance().extendNU(Constants.Arm.LOW_EXTEND_NU);
                targetPos = Constants.Arm.LOW_EXTEND_NU;
                targetPivot = Constants.Arm.LOW_ANGLE * multiplier;
                targetWrist = Constants.Grabber.PREP_CONE_FRONT_NU * multiplier;
                break;
        }
        gotToStart = false;

        atStartDeg = Math.abs(ArmSubsystem.getInstance().getPivotRad() - Constants.Arm.PREP_ANGLE) < 1*Constants.TAU/360;

        //On-the-field Diagnostic information
        Tabs.Comp.setPivotTarget(targetPivot);
        Tabs.Comp.setExtendTarget(targetPos);
        Tabs.Comp.setWristTarget(targetWrist);

        if(Robot.state == RobotState.OK && ArmSubsystem.getInstance().pivotStopped()) ArmSubsystem.getInstance().resetPivotNU();
    }

    @Override
    public void execute() {
        //Manual Extension:
        //Make sure that the arm has reached its desired extend position before we allow manual movement to happen
        if(!DriverStation.isAutonomous() && !atStartDeg && Math.abs(targetPivot) < Constants.TAU/4){
            ArmSubsystem.getInstance().pivot(Constants.Arm.PREP_ANGLE * multiplier);
            ArmSubsystem.getInstance().extendNU(Constants.Arm.EXTEND_REST_NU);
            if(Math.abs(Math.abs(ArmSubsystem.getInstance().getPivotRad()) - Constants.Arm.PREP_ANGLE) < 1*Constants.TAU/360){
                ArmSubsystem.getInstance().pivot(targetPivot);
                ArmSubsystem.getInstance().extendNU(targetPos);
                atStartDeg = true;
            }
        }
        else{
            if(!gotToStart && Math.abs(ArmSubsystem.getInstance().getExtendNU() - targetPos) < Constants.Arm.EXTEND_TARGET_DEADZONE) gotToStart = true;
            if(gotToStart){
                double y = JoystickSubsystem.getInstance().getManualExtend();   //Deadzone math
                if(y < 0) y *= Constants.Joystick.MANUAL_EXTEND_OUT_SENSITIVITY; //Negative is extend out
                else if(y > 0) y *= Constants.Joystick.MANUAL_EXTEND_IN_SENSITIVITY;    //Positive is retract in
                
                if(y == 0){ // If there isn't any input, maintain the position
                    if(!extensionMan0){ //When the joystick is first zeroed
                        extensionMan0 = true;   //Flag variable to see if the joystick was zeroed
                        lastPos = ArmSubsystem.getInstance().getExtendNU();
                        // ^ Without this, the arm goes in bit by bit because gravity, lastPos reads that position and tells the extension to go there
                    }
                    ArmSubsystem.getInstance().extendNU(lastPos);
                }
                else{   //If the joystick is moving, we move the arm at a percent output
                    ArmSubsystem.getInstance().set(-y);
                    extensionMan0 = false;  //The joystick is not zero, so we are moving
                }

                //Manual Pivot:
                //All of the logic is the same as the extension
                if(Math.abs(ArmSubsystem.getInstance().getPivotRad() - targetPivot) < Constants.Arm.PIVOT_TARGET_DEADZONE){
                    atPivot = true;
                } 
        
                if(atPivot) {
                    double pivotX = JoystickSubsystem.getInstance().getManualPivot();
                    if(pivotX == 0){ // If there isn't any input, maintain the position
                        if(!pivotMan0){
                            pivotMan0 = true;
                            lastPivot = ArmSubsystem.getInstance().getPivotRad();
                        }
                        ArmSubsystem.getInstance().pivot(lastPivot);
                    }
                    else{
                        ArmSubsystem.getInstance().setPivot(pivotX*multiplier);
                        pivotMan0 = false;
                    }
                }

                if(RobotContainer.operatorController.pov(0).getAsBoolean()) targetWrist -= Constants.Joystick.MANUAL_WRIST_SENSITIVITY * multiplier;
                if(RobotContainer.operatorController.pov(180).getAsBoolean()) targetWrist += Constants.Joystick.MANUAL_WRIST_SENSITIVITY * multiplier;

                GrabberSubsystem.getInstance().orientPos(targetWrist);

                // if(!resetEncoder && 
                //     Math.abs(ArmSubsystem.getInstance().getPivotRad()-targetPivot) <= 5 * Constants.TAU/360 && 
                //     Math.abs(ArmSubsystem.getInstance().getPivotSpeed()) < 10){
                //     ArmSubsystem.getInstance().resetPivotNU();
                //     resetEncoder = true;
                // }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}

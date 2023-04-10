package frc.robot.commands.score;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tabs;
import frc.robot.lib.math.NRUnits;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PrepHighConeCommand extends CommandBase{
    double multiplier;
    double l0 = 0.690;

    double targetPos;
    double targetPivot;
    double targetWrist;
    

    double lastPos;
    double lastPivot;
    boolean extensionMan0;
    boolean pivotMan0;
    boolean gotToStart;
    boolean atPivot;

    boolean resetEncoder;

    public PrepHighConeCommand(){
        addRequirements(ArmSubsystem.getInstance());
    }

    // public double NUtoMPS(double NU){
    //     //Converts NU/100ms to meters/s
    //     NU *= 10;
    //     NU /= 58.4;
    //     NU /= 1000;

    //     return NU;
    // }

    // public double mpsToNU(double mps){
    //     mps *= 1000;
    //     mps *= 58.4;
    //     mps /= 10;

    //     return mps;
    // }

    // private double NUToRadiansPerSecond(double NU) {
    //     NU = NRUnits.Pivot.NUToRad(NU);
    //     return NU * 10;
    // }

    @Override
    public void initialize() {
        lastPos = 0;
        lastPivot = 0;
        extensionMan0 = false;
        pivotMan0 = false;
        gotToStart = false;
        atPivot = false;

        double gyroAngle = SwerveDriveSubsystem.getInstance().getGyroAngle();

        boolean scoreFront = gyroAngle > Constants.TAU/4 || gyroAngle < -Constants.TAU/4;

        multiplier = scoreFront ? 1 : -1;

        double prepAngle = 0;
        if(multiplier == 1) {
            prepAngle = Constants.Arm.HIGH_FRONT_ANGLE + (0)*Constants.TAU/360;
            ArmSubsystem.getInstance().setPivotCruiseVelocity(50_000);
            ArmSubsystem.getInstance().setPivotAcceleration(55_000);

            ArmSubsystem.getInstance().setExtendAcceleration(50_000);
        }
        else {
            prepAngle = -(Constants.Arm.HIGH_FRONT_ANGLE - 1.1*Constants.TAU/360);
            ArmSubsystem.getInstance().setPivotCruiseVelocity(50_000);
            ArmSubsystem.getInstance().setPivotAcceleration(35_000);

            ArmSubsystem.getInstance().setExtendAcceleration(30_000);
        }

        ArmSubsystem.getInstance().pivot(prepAngle);
        ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU);
        GrabberSubsystem.getInstance().orientPos(-8.5 * multiplier);

        

        targetPos = Constants.Arm.HIGH_EXTEND_NU;
        targetPivot = prepAngle;
        targetWrist = Constants.Grabber.PREP_CONE_FRONT_NU * multiplier;

        Tabs.Comp.setExtendTarget(targetPos);
        Tabs.Comp.setPivotTarget(targetPivot);
        Tabs.Comp.setWristTarget(targetWrist);
    }

    @Override
    public void execute() {
        //l0 = 0.690m
        double pivotAngle = ArmSubsystem.getInstance().getAngle();
        double pivotSpeed = ArmSubsystem.getInstance().getPivotSpeed();
        pivotSpeed = NRUnits.Pivot.NUToRPS(pivotSpeed);

        double extendSpeed = Constants.Arm.l0 * Math.tan(pivotAngle)/Math.cos(pivotAngle)*pivotSpeed;
        extendSpeed = NRUnits.Extension.mpsToNU(extendSpeed);

        if(multiplier == 1) ArmSubsystem.getInstance().setExtendCruiseVelocity(extendSpeed + 6000);
        else ArmSubsystem.getInstance().setExtendCruiseVelocity(extendSpeed);

        //NOTE: Could potentially causes issues when manuallying in
        if(Math.abs(ArmSubsystem.getInstance().getExtendNU() - Constants.Arm.HIGH_EXTEND_NU) < 1000){   //While extending, we move the wrist to get out of the way of the bottom node
            GrabberSubsystem.getInstance().orientPos(Constants.Grabber.PREP_CONE_FRONT_NU * multiplier);          //Once we've reached a close enough position, then put the wrist into scoring position
        }


        //MANUAL!!!!!!!!!!!!!!!!!
        if(!gotToStart && Math.abs(ArmSubsystem.getInstance().getPos() - targetPos) < Constants.Arm.EXTEND_TARGET_DEADZONE) gotToStart = true;
        if(gotToStart) {
            double y = JoystickSubsystem.getInstance().getManualExtend();
            if(y < 0) y *= Constants.Joystick.MANUAL_EXTEND_OUT_SENSITIVITY;
            else y *= Constants.Joystick.MANUAL_EXTEND_IN_SENSITIVITY;
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
                double pivotX = JoystickSubsystem.getInstance().getManualPivot();
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
        }

        if(!resetEncoder && Math.abs(ArmSubsystem.getInstance().getAngle()-targetPivot) <= Constants.Arm.HIGH_FRONT_ANGLE && Math.abs(ArmSubsystem.getInstance().getPivotSpeed()) < 100){
            ArmSubsystem.getInstance().resetPivotNU();
            resetEncoder = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        //TODO: Why is this here? This should be empty: Test and make sure it still works with this gone.

        // ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        // ArmSubsystem.getInstance().setDefaultAcceleration();

        // ArmSubsystem.getInstance().setExtendCruiseVelocity(Constants.Arm.ARM_CRUISE_VELOCITY);
        // ArmSubsystem.getInstance().setExtendAcceleration(15_000);

        // ArmSubsystem.getInstance().pivot(0);
        // ArmSubsystem.getInstance().extendNU(0);
        // GrabberSubsystem.getInstance().orientPos(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

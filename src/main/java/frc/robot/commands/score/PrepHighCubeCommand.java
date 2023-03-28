package frc.robot.commands.score;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tabs;
import frc.robot.lib.math.NRUnits;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsytem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PrepHighCubeCommand extends CommandBase{
    double l0 = 0.690;
    double targetPos;
    double targetPivot;
    double wristPos;
    double multiplier;

    double lastPos;
    double lastPivot;
    boolean extendMan0;
    boolean pivotMan0;
    boolean gotToStart;
    boolean atPivot;

    boolean resetEncoder;

    public PrepHighCubeCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        lastPos = 0;
        lastPivot = 0;
        extendMan0 = false;
        pivotMan0 = false;
        gotToStart = false;
        atPivot = false;

        double gyroAngle = SwerveDriveSubsystem.getInstance().getGyroAngle();

        boolean scoreFront = gyroAngle > Constants.TAU/4 || gyroAngle < -Constants.TAU/4;

        multiplier = 1;

        double prepAngle = Constants.Arm.Cube.HIGH_ANGLE;   //Right now, we only score from one side because...

        // if(multiplier == 1) prepAngle = Constants.Arm.Cube.HIGH_ANGLE + (1)*Constants.TAU/360;
        // else prepAngle = -(Constants.Arm.Cube.HIGH_ANGLE - 0.1*Constants.TAU/360);

        ArmSubsystem.getInstance().pivot(prepAngle);
        ArmSubsystem.getInstance().extendNU(Constants.Arm.Cube.HIGH_EXTEND_NU);
        GrabberSubsystem.getInstance().orientPos(Constants.Grabber.CUBE_NU);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(50_000);
        ArmSubsystem.getInstance().setPivotAcceleration(55_000);

        ArmSubsystem.getInstance().setExtendAcceleration(50_000);

        targetPos = Constants.Arm.Cube.HIGH_EXTEND_NU;
        targetPivot = prepAngle;
        wristPos = Constants.Grabber.CUBE_NU;

        resetEncoder = false;

        Tabs.Comp.setExtendTarget(targetPos);
        Tabs.Comp.setPivotTarget(targetPivot);
        Tabs.Comp.setWristTarget(wristPos);
    }

    @Override
    public void execute() {
        double pivotAngle = ArmSubsystem.getInstance().getAngle();
        double pivotSpeed = ArmSubsystem.getInstance().getPivotSpeed();
        pivotSpeed = NRUnits.Pivot.NUToRPS(pivotSpeed);

        double extendSpeed = l0 * Math.tan(pivotAngle)/Math.cos(pivotAngle)*pivotSpeed;
        extendSpeed = NRUnits.Extension.mpsToNU(extendSpeed);

        ArmSubsystem.getInstance().setExtendCruiseVelocity(extendSpeed+5_000);  //NOTE: the +5_000 might be why it tips

        //MANUAL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! AAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHH

        if(!gotToStart && Math.abs(ArmSubsystem.getInstance().getPos() - targetPos) < Constants.Arm.EXTEND_TARGET_DEADZONE) gotToStart = true;
        if(gotToStart) {
            double y = JoystickSubsytem.getInstance().getManualExtend();
            if(y < 0) y *= Constants.Joystick.MANUAL_EXTEND_OUT_SENSITIVITY;
            else y *= Constants.Joystick.MANUAL_EXTEND_IN_SENSITIVITY;
            if(y == 0){ // If there isn't any input, maintain the position
                if(!extendMan0){
                    extendMan0 = true;
                    lastPos = ArmSubsystem.getInstance().getPos();
                }
                ArmSubsystem.getInstance().extendNU(lastPos);
            }
            else{
                ArmSubsystem.getInstance().set(-y);
                extendMan0 = false;
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

            if(RobotContainer.operatorController.pov(0).getAsBoolean()) wristPos -= Constants.Joystick.MANUAL_WRIST_SENSITIVITY * multiplier;
            if(RobotContainer.operatorController.pov(180).getAsBoolean()) wristPos += Constants.Joystick.MANUAL_WRIST_SENSITIVITY * multiplier;

            GrabberSubsystem.getInstance().orientPos(wristPos);

            if(!resetEncoder && Math.abs(ArmSubsystem.getInstance().getAngle()-targetPivot) <= Constants.Arm.INTAKE_DEADZONE){
                ArmSubsystem.getInstance().resetPivotNU();
                resetEncoder = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

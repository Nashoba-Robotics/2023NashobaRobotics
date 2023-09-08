package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tabs;
import frc.robot.Robot.RobotState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

public class IntakeCubeCommand extends CommandBase {
    boolean pivotMan0;
    double lastPivot;

    double targetPivot;
    boolean atPivot;

    boolean to90 = false;

    boolean resetEncoder;

    public IntakeCubeCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }
    public IntakeCubeCommand(boolean to90){
        this.to90 = to90;
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        // GrabberSubsystem.getInstance().setCurrentLimit(30);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(Constants.Arm.INTAKE_PIVOT_VELOCITY);
        ArmSubsystem.getInstance().setPivotAcceleration(Constants.Arm.INTAKE_PIVOT_ACCELERATION);

        // Extend is TEMP to test at the same distance
        ArmSubsystem.getInstance().extendNU(Constants.Arm.Cube.INTAKE_EXTEND_NU);
        ArmSubsystem.getInstance().pivot(Constants.Arm.Cube.INTAKE_ANGLE);
        targetPivot = Constants.Arm.Cube.INTAKE_ANGLE;
        atPivot = false;
        pivotMan0 = false;
        GrabberSubsystem.getInstance().orientPos(Constants.Grabber.CUBE_NU);
        lastPivot = ArmSubsystem.getInstance().getPivotRad();

        resetEncoder = false;

        Tabs.Comp.setPivotTarget(targetPivot);
        Tabs.Comp.setExtendTarget(Constants.Arm.Cube.INTAKE_EXTEND_NU);
        Tabs.Comp.setWristTarget(Constants.Grabber.CUBE_NU);
    }

    @Override
    public void execute() {
        GrabberSubsystem.getInstance().set(Constants.Grabber.CUBE_INTAKE_SPEED);
        // SmartDashboard.putNumber("Grabber Current", GrabberSubsystem.getInstance().getCurrent());

        if(Math.abs(ArmSubsystem.getInstance().getPivotRad() - targetPivot) < Constants.Arm.PIVOT_TARGET_DEADZONE){
            atPivot = true;
            lastPivot = ArmSubsystem.getInstance().getPivotRad();
        } 

        if(atPivot) {
            double pivotX = JoystickSubsystem.getInstance().getManualPivot();
            if(pivotX == 0){ // If there isn't any input, maintain the position
                // ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getAngle());
                if(!pivotMan0){
                    pivotMan0 = true;
                    lastPivot = ArmSubsystem.getInstance().getPivotRad();
                }
                ArmSubsystem.getInstance().pivot(lastPivot);
            }
            else{
                ArmSubsystem.getInstance().setPivot(pivotX);
                pivotMan0 = false;
            }
        }

        if(Robot.state != RobotState.OK && !resetEncoder && 
        ArmSubsystem.getInstance().pivotStopped() && 
        Math.abs(ArmSubsystem.getInstance().getPivotRad()-Constants.Arm.Cube.INTAKE_ANGLE) <= Constants.Arm.INTAKE_DEADZONE){
            if(Math.abs(ArmSubsystem.getInstance().getPivotRad()) > Constants.TAU/4) {
                // ArmSubsystem.getInstance().resetPivotNU();
                resetEncoder = true;
            } else {
                // CandleSubsystem.getInstance().set(CandleState.BAD);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // GrabberSubsystem.getInstance().setCurrentLimit(10);
        // if(Robot.state == RobotState.OK && ArmSubsystem.getInstance().pivotStopped()) ArmSubsystem.getInstance().resetPivotNU();
        if(!to90)ArmSubsystem.getInstance().pivot(0);
        else ArmSubsystem.getInstance().pivot(-Constants.TAU/8);
        GrabberSubsystem.getInstance().set(Constants.Grabber.CUBE_HOLD_SPEED);   //Make the grabber hold it
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

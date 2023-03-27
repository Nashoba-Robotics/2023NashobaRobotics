package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsytem;

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
        GrabberSubsystem.getInstance().setCurrentLimit(30);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(40_000);
        ArmSubsystem.getInstance().setPivotAcceleration(40_000);

        // Extend is TEMP to test at the same distance
        ArmSubsystem.getInstance().extendNU(3_000);
        ArmSubsystem.getInstance().pivot(Constants.Arm.Cube.INTAKE_ANGLE);
        targetPivot = Constants.Arm.Cube.INTAKE_ANGLE;
        atPivot = false;
        pivotMan0 = false;
        GrabberSubsystem.getInstance().orientPos(Constants.Grabber.CUBE_NU);
        lastPivot = ArmSubsystem.getInstance().getAngle();

        resetEncoder = false;
    }

    @Override
    public void execute() {
        GrabberSubsystem.getInstance().set(Constants.Grabber.CUBE_RELEASE_SPEED, -Constants.Grabber.CUBE_RELEASE_SPEED);
        // SmartDashboard.putNumber("Grabber Current", GrabberSubsystem.getInstance().getCurrent());

        if(Math.abs(ArmSubsystem.getInstance().getAngle() - targetPivot) < Constants.Arm.PIVOT_TARGET_DEADZONE){
            atPivot = true;
            lastPivot = ArmSubsystem.getInstance().getAngle();
        } 

        if(atPivot) {
            double pivotX = JoystickSubsytem.getInstance().getManualPivot();
            if(pivotX == 0){ // If there isn't any input, maintain the position
                // ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getAngle());
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

        if(!resetEncoder && Math.abs(ArmSubsystem.getInstance().getAngle()-Constants.Arm.INTAKE_ANGLE) <= Constants.Arm.INTAKE_DEADZONE){
            ArmSubsystem.getInstance().resetPivotNU();
            resetEncoder = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().setCurrentLimit(10);
        if(!to90)ArmSubsystem.getInstance().pivot(0);
        else ArmSubsystem.getInstance().pivot(-Constants.TAU/4);
        GrabberSubsystem.getInstance().set(Constants.Grabber.CUBE_HOLD_SPEED, -Constants.Grabber.CUBE_HOLD_SPEED);   //Make the grabber hold it
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

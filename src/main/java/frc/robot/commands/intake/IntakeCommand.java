package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tabs;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsytem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

public class IntakeCommand extends CommandBase {
    double extendNU = 3_000;
    
    boolean pivotMan0;
    double lastPivot;

    double pivotTarget;
    boolean atPivot;

    boolean resetEncoder;   // Right now, we are only going to read the absolute encoder twice
                            // First time is at the beginnig, and the second time is when the arm reaches intake angle

    int multiplier;

    public IntakeCommand(boolean intakeFront){
        multiplier = intakeFront ? 1 : -1;
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        GrabberSubsystem.getInstance().setCurrentLimit(30);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(400_000); //<-- Make sure we are limited by Acceleration
        ArmSubsystem.getInstance().setPivotAcceleration(60_000);

        ArmSubsystem.getInstance().extendNU(extendNU);
        ArmSubsystem.getInstance().pivot(Constants.Arm.INTAKE_ANGLE * multiplier);
        pivotTarget = Constants.Arm.INTAKE_ANGLE * multiplier;
        atPivot = false;
        pivotMan0 = false;
        GrabberSubsystem.getInstance().orientPos(Constants.Grabber.INTAKE_ANGLE * multiplier);
        lastPivot = ArmSubsystem.getInstance().getAngle();

        resetEncoder = false;

        ArmSubsystem.getInstance().resetPivotNU();
        LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.CONE_CAM); //Can get rid of this after adding USB cam

        Tabs.Comp.setPivotTarget(pivotTarget);
        Tabs.Comp.setExtendTarget(extendNU);
        Tabs.Comp.setWristTarget(Constants.Grabber.INTAKE_ANGLE);
    }

    @Override
    public void execute() {
        GrabberSubsystem.getInstance().intake();
        SmartDashboard.putNumber("Arm Angle Deg", ArmSubsystem.getInstance().getAngle()*360/Constants.TAU);

        if(Math.abs(ArmSubsystem.getInstance().getAngle() - pivotTarget) < 0.5 * Constants.TAU/360){
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

        SmartDashboard.putNumber("Top Stator", GrabberSubsystem.getInstance().getTopGrabCurrent());
        if(GrabberSubsystem.getInstance().getTopGrabCurrent() > 30) {
            // GrabberSubsystem.getInstance().setCurrentLimit(10);
            // GrabberSubsystem.getInstance().set(-0.1);
            CandleSubsystem.getInstance().set(CandleState.HAVE_CONE);
        }
        else{
            CandleSubsystem.getInstance().set(CandleState.WANT_CONE);
        }

        if(!resetEncoder && Math.abs(ArmSubsystem.getInstance().getAngle()-Constants.Arm.INTAKE_ANGLE) <= Constants.Arm.INTAKE_DEADZONE){
            ArmSubsystem.getInstance().resetPivotNU();
            resetEncoder = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().resetPivotNU();
        GrabberSubsystem.getInstance().setCurrentLimit(10);
        ArmSubsystem.getInstance().pivot(0);
        GrabberSubsystem.getInstance().orient(0);
        GrabberSubsystem.getInstance().set(Constants.Grabber.CONE_HOLD_SPEED);   //Make the grabber hold it
        
        // LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.APRIL_TAG_PIPELINE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tabs;
import frc.robot.Robot.RobotState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

public class IntakeConeCommand extends CommandBase {
    boolean pivotMan0;
    double lastPivot;

    double pivotTarget;
    boolean atPivot;

    boolean resetEncoder;   // Right now, we are only going to read the absolute encoder twice
                            // First time is at the beginnig, and the second time is when the arm reaches intake angle

    int multiplier;

    //Data for seeing if we have a cone in the intake
    double currentThreshold = 28;
    boolean timerStarted = false;
    Timer timer;

    public IntakeConeCommand(boolean intakeFront){
        multiplier = intakeFront ? 1 : -1;
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());

        timer = new Timer();
    }

    @Override
    public void initialize() {
        // GrabberSubsystem.getInstance().setCurrentLimit(30);
        GrabberSubsystem.getInstance().setCurrentLimit(true);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(100); //<-- Make sure we are limited by Acceleration
        ArmSubsystem.getInstance().setPivotAcceleration(293);

        ArmSubsystem.getInstance().extendNU(Constants.Arm.INTAKE_EXTEND_NU);
        ArmSubsystem.getInstance().pivot(Constants.Arm.INTAKE_ANGLE * multiplier);
        pivotTarget = Constants.Arm.INTAKE_ANGLE * multiplier;
        atPivot = false;
        pivotMan0 = false;
        GrabberSubsystem.getInstance().orientPos(Constants.Grabber.INTAKE_ANGLE * multiplier);
        lastPivot = ArmSubsystem.getInstance().getPivotRad();

        resetEncoder = false;

        if(Robot.state == RobotState.OK && ArmSubsystem.getInstance().pivotStopped()) ArmSubsystem.getInstance().resetPivotNU();
        LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.CONE_CAM); //Can get rid of this after adding USB cam

        Tabs.Comp.setPivotTarget(pivotTarget);
        Tabs.Comp.setExtendTarget(Constants.Arm.INTAKE_EXTEND_NU);
        Tabs.Comp.setWristTarget(Constants.Grabber.INTAKE_ANGLE);
    }

    @Override
    public void execute() {
        GrabberSubsystem.getInstance().intake();

        if(Math.abs(ArmSubsystem.getInstance().getPivotRad() - pivotTarget) < 0.5 * Constants.TAU/360){
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
                ArmSubsystem.getInstance().setPivot(pivotX);
                pivotMan0 = false;
            }
        }

        //LED stuff
        if(Robot.state != RobotState.OK){
            CandleSubsystem.getInstance().set(CandleState.SYSTEM_BAD);
        }
        else if(GrabberSubsystem.getInstance().getGrabberCurrent() > 30) {
            if(!timerStarted){
                timerStarted = true;
                timer.start();
            }
            if(timer.get() >= 0.5){
                CandleSubsystem.getInstance().set(CandleState.HAVE_CONE);
                timer.stop();   //Don't remember if resetting the timers stops it.
                timer.reset();
                
                timerStarted = false;
            }
        }
        else{//When the current is outside of the threshold, we want to stop the lights and reset the timer
            CandleSubsystem.getInstance().set(CandleState.WANT_CONE);

            timer.stop();   
            timer.reset();
                
            timerStarted = false;
        }

        if(Robot.state != RobotState.OK && !resetEncoder && 
        ArmSubsystem.getInstance().pivotStopped() && 
        Math.abs(ArmSubsystem.getInstance().getPivotRad()-Constants.Arm.INTAKE_ANGLE) <= Constants.Arm.INTAKE_DEADZONE){
            if(Math.abs(ArmSubsystem.getInstance().getPivotRad()) > Constants.TAU/4) {
                ArmSubsystem.getInstance().resetPivotNU();
                resetEncoder = true;
            } else {
                CandleSubsystem.getInstance().set(CandleState.BAD);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(Robot.state == RobotState.OK && ArmSubsystem.getInstance().pivotStopped()) ArmSubsystem.getInstance().resetPivotNU();
        GrabberSubsystem.getInstance().setCurrentLimit(25, 25, 0.1);
        ArmSubsystem.getInstance().pivot(0);
        // GrabberSubsystem.getInstance().orient(0);
        // GrabberSubsystem.getInstance().set(Constants.Grabber.CONE_HOLD_SPEED);   //Make the grabber hold it   
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

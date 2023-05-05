package frc.robot.commands.auto.systemcheck;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

/*
 * Right now, this is time based. After you feed a cone to the intake, it will wait 500ms and then go back up
 * Can turn this into a button press later on if the current reading doesn't work
 */
public class IntakeConeCheck extends CommandBase{
    double extendNU = 3_000;
    
    boolean pivotMan0;
    double lastPivot;

    double pivotTarget;
    boolean atPivot;

    boolean resetEncoder;   // Right now, we are only going to read the absolute encoder twice
                            // First time is at the beginnig, and the second time is when the arm reaches intake angle

    double currentThreshold = 28;
    boolean timerStarted = false;
    Timer timer;

    boolean haveCone;
    boolean finish;

    public IntakeConeCheck(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());

        timer = new Timer();
    }

    @Override
    public void initialize() {
        GrabberSubsystem.getInstance().setCurrentLimit(true);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(30_000);
        ArmSubsystem.getInstance().setPivotAcceleration(60_000);

        ArmSubsystem.getInstance().extendNU(extendNU);
        ArmSubsystem.getInstance().pivot(Constants.Arm.INTAKE_ANGLE);
        pivotTarget = Constants.Arm.INTAKE_ANGLE;
        atPivot = false;
        pivotMan0 = false;
        GrabberSubsystem.getInstance().orientPos(Constants.Grabber.INTAKE_ANGLE);
        lastPivot = ArmSubsystem.getInstance().getAngle();

        resetEncoder = false;

        ArmSubsystem.getInstance().resetPivotNU();

        haveCone = false;
        finish = false;
    }

    @Override
    public void execute() {
        GrabberSubsystem.getInstance().intake();

        if(GrabberSubsystem.getInstance().getGrabberCurrent() > 30) {
            if(haveCone && !timerStarted){
                timerStarted = true;
                timer.start();
            }
            else if(!haveCone && !timerStarted){
                timerStarted = true;
                timer.start();
            }
            //                                 v Time after taking in the cone for the arm to go back up
            else if(haveCone && timer.get() >= 0.5){
                finish = true;
            }
            else if(!haveCone && timer.get() >= 0.5){
                CandleSubsystem.getInstance().set(CandleState.PARTIAL_CHECK_1);

                timer.stop();
                timer.reset();
                
                timerStarted = false;
                haveCone = true;
            }
        }

        //Check if the arm pivot speed is 0
        if(!resetEncoder && 
        Math.abs(ArmSubsystem.getInstance().getPivotSpeed()) < 3.0 && 
        Math.abs(ArmSubsystem.getInstance().getAngle()-Constants.Arm.INTAKE_ANGLE) <= Constants.Arm.INTAKE_DEADZONE){
            if(Math.abs(ArmSubsystem.getInstance().getAngle()) > Constants.TAU/4) {
                ArmSubsystem.getInstance().resetPivotNU();
                resetEncoder = true;
            } else {
                CandleSubsystem.getInstance().set(CandleState.BAD);
                SmartDashboard.putString("ENCODER ERROR", "Intake Failed");
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().resetPivotNU();
        GrabberSubsystem.getInstance().setCurrentLimit(25, 25, 0.1);
        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(0);

        CandleSubsystem.getInstance().set(CandleState.SYSTEM_GOOD);
    }

    @Override
    public boolean isFinished() {
        return finish;
    }
}

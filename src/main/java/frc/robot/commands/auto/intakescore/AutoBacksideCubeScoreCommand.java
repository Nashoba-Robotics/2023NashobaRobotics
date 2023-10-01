package frc.robot.commands.auto.intakescore;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoBacksideCubeScoreCommand extends CommandBase{
    public boolean high;
    public double extendLength;

    private boolean extended;
    private boolean scored;

    private double extendTime = 1;
    private double scoreTime = 1;

    double startTime;

    public AutoBacksideCubeScoreCommand(boolean high){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
        this.high = high;

        if(high){
            extendLength = Constants.Arm.Cube.HIGH_EXTEND_NU;
            extendTime = 0.7;
            scoreTime = 0.3;
        }
        else{
            extendLength = Constants.Arm.Cube.MID_EXTEND_NU-5;
            extendTime = 0.7;
            scoreTime = 0.3;
        }
    }

    // Assume the robot is already at the correct cube angle
    @Override
    public void initialize() {
        extended = false;
        scored = false;
        startTime = System.currentTimeMillis()/1000.;
        ArmSubsystem.getInstance().setExtendCruiseVelocity(100);
        ArmSubsystem.getInstance().setExtendAcceleration(200);

        ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getPivotRad());
    }

    @Override
    public void execute() {
        if(!extended && !scored){
            ArmSubsystem.getInstance().extendNU(extendLength);
            if(System.currentTimeMillis()/1000.-startTime > extendTime){
                extended = true;
                startTime = System.currentTimeMillis()/1000.;
            }
        } 
        else if(!scored){
            GrabberSubsystem.getInstance().set(Constants.Grabber.CUBE_RELEASE_SPEED);
            if(System.currentTimeMillis()/1000.-startTime > scoreTime){
                scored = true;
                startTime = System.currentTimeMillis()/1000.;
                GrabberSubsystem.getInstance().set(0);
            }
        } 
        else ArmSubsystem.getInstance().extendNU(Constants.Arm.EXTEND_REST_NU);

        
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().setPivotCruiseVelocity(0.35);
        ArmSubsystem.getInstance().setPivotAcceleration(0.7);
        ArmSubsystem.getInstance().pivot(0);
    }

    @Override
    public boolean isFinished() {
        return scored && extended && (System.currentTimeMillis()/1000-startTime > 0.1);
    }
}

package frc.robot.commands.auto.intakescore;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoBacksideCubeScoreCommand extends CommandBase{
    boolean high;
    double extendLength;

    boolean extended = false;
    boolean scored = true;

    double extendTime = 1;
    double scoredTime = 1;

    double startTime;

    public AutoBacksideCubeScoreCommand(boolean high){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
        this.high = high;

        if(high){
            extendLength = Constants.Arm.Cube.HIGH_EXTEND_NU;
        }
        else{
            extendLength = Constants.Arm.Cube.MID_EXTEND_NU;
        }
    }

    // Assume the robot is already at the correct prep angle
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis()/1000;
        ArmSubsystem.getInstance().setExtendCruiseVelocity(100);
        ArmSubsystem.getInstance().setExtendAcceleration(200);
    }

    @Override
    public void execute() {
        if(!extended && !scored){
            ArmSubsystem.getInstance().extendNU(extendLength);
            if(System.currentTimeMillis()/1000 > 1){
                extended = true;
                startTime = System.currentTimeMillis()/1000;
            }
        } 
        else if(!scored){
            GrabberSubsystem.getInstance().set(Constants.Grabber.CUBE_RELEASE_SPEED);
            if(System.currentTimeMillis()/1000 > 1){
                scored = true;
                startTime = System.currentTimeMillis()/1000;
            }
        } 
        else ArmSubsystem.getInstance().extendNU(Constants.Arm.EXTEND_REST_NU);

        
    }

    @Override
    public boolean isFinished() {
        return scored && extended;
    }
}

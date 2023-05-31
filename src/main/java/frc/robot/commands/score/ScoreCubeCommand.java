package frc.robot.commands.score;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;


public class ScoreCubeCommand extends CommandBase {
    double startTime;
    public ScoreCubeCommand() {
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        GrabberSubsystem.getInstance().setCurrentLimit(45, 45, 0.1);    //Test and make sure that this still works
        GrabberSubsystem.getInstance().set(Constants.Grabber.CUBE_RELEASE_SPEED);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        
        
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().setCurrentLimit(true);
        if(!DriverStation.isAutonomous()){
            ArmSubsystem.getInstance().setPivotCruiseVelocity(100);
            ArmSubsystem.getInstance().setPivotAcceleration(150);
            ArmSubsystem.getInstance().setExtendCruiseVelocity(100);
            ArmSubsystem.getInstance().setExtendAcceleration(300);  //There is no way in living hell that this is correct
            GrabberSubsystem.getInstance().set(0);
            GrabberSubsystem.getInstance().orient(0);
            
            ArmSubsystem.getInstance().stop();
            ArmSubsystem.getInstance().pivot(0);
            ArmSubsystem.getInstance().extendNU(Constants.Arm.EXTEND_REST_NU);
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > 400;
    }

}

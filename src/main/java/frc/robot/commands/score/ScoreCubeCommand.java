package frc.robot.commands.score;

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
        // GrabberSubsystem.getInstance().setCurrentLimit(40);
        // GrabberSubsystem.getInstance().setCurrentLimit(false);
        GrabberSubsystem.getInstance().setCurrentLimit(45, 45, 0.1);
        GrabberSubsystem.getInstance().set(Constants.Grabber.CUBE_RELEASE_SPEED);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        
        
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().setCurrentLimit(true);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(30_000);
        ArmSubsystem.getInstance().setPivotAcceleration(30_000);
        ArmSubsystem.getInstance().setExtendCruiseVelocity(50_000);
        ArmSubsystem.getInstance().setExtendAcceleration(50_000);
        GrabberSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().orient(0);
        
        ArmSubsystem.getInstance().stop();
        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(3_000);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > 400;
    }

}

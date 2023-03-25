package frc.robot.commands.score;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;


public class ScoreCubeCommand extends CommandBase {
    public ScoreCubeCommand() {
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        GrabberSubsystem.getInstance().setCurrentLimit(40);
        GrabberSubsystem.getInstance().set(-0.4, 0.4);
    }

    @Override
    public void execute() {
        
        
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().setPivotCruiseVelocity(50_000);
        ArmSubsystem.getInstance().setPivotAcceleration(50_000);
        GrabberSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().orient(0);

        ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        ArmSubsystem.getInstance().setDefaultAcceleration();
        
        ArmSubsystem.getInstance().stop();
        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

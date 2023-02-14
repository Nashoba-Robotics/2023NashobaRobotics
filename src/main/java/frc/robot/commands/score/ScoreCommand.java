package frc.robot.commands.score;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ScoreCommand extends CommandBase {
    
    private long startTime;    

    public ScoreCommand() {
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getAngle() + 5 * Constants.TAU / 360);
        ArmSubsystem.getInstance().extendNU(1000);
        GrabberSubsystem.getInstance().set(0.1);
    }

    @Override
    public void execute() {
        
        
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().orient(0);
        ArmSubsystem.getInstance().stop();
        ArmSubsystem.getInstance().pivot(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(ArmSubsystem.getInstance().getPos()-100) < 100 || System.currentTimeMillis() - startTime > 5000;
    }

}

package frc.robot.commands.score;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class LowScoreCommand extends CommandBase {
    private long startTime;    

    public LowScoreCommand() {
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getAngle());
        ArmSubsystem.getInstance().extendNU(1000);
        GrabberSubsystem.getInstance().set(0.3);
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
        return System.currentTimeMillis() - startTime > 5000;
    }

}
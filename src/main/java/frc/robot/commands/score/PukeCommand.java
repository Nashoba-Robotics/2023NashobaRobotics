package frc.robot.commands.score;

import javax.naming.InitialContext;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Grabber;
import frc.robot.subsystems.GrabberSubsystem;

public class PukeCommand extends CommandBase{

    public PukeCommand(){
        addRequirements(GrabberSubsystem.getInstance());
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        GrabberSubsystem.getInstance().setOrientSpeed(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().orient(0);;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

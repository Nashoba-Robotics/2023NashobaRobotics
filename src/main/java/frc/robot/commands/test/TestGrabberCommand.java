package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class TestGrabberCommand extends CommandBase {
    
    public TestGrabberCommand() {
        addRequirements(GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        GrabberSubsystem.getInstance().zeroWrist();
        SmartDashboard.putNumber("Orient Angle", 0);
    }

    @Override
    public void execute() {
        GrabberSubsystem.getInstance().orient(SmartDashboard.getNumber("Orient Angle", 0));
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

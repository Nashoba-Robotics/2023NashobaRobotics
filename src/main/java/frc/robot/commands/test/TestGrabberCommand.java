package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Grabber;
import frc.robot.subsystems.GrabberSubsystem;

public class TestGrabberCommand extends CommandBase {
    
    public TestGrabberCommand() {
        addRequirements(GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        GrabberSubsystem.getInstance().zeroWrist();
        SmartDashboard.putNumber("Orient Angle", 0);
        SmartDashboard.putNumber("Wrist Speed", 0);
    }

    double speed;
    @Override
    public void execute() {
        //GrabberSubsystem.getInstance().orient(SmartDashboard.getNumber("Orient Angle", 0));
        speed = SmartDashboard.getNumber("Wrist Speed", 0.1);
        //GrabberSubsystem.getInstance().setOrientSpeed(speed);
        GrabberSubsystem.getInstance().orient(speed);
        SmartDashboard.putNumber("Wrist Pos", GrabberSubsystem.getInstance().getOrientPos());
        SmartDashboard.putNumber("Num", speed);
        SmartDashboard.putNumber("Hello", 15);
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

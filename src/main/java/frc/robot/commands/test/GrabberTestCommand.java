package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Tabs;
import frc.robot.subsystems.GrabberSubsystem;

public class GrabberTestCommand extends CommandBase{
    public GrabberTestCommand(){
        addRequirements(GrabberSubsystem.getInstance());
    }
    @Override
    public void execute() {
        double speed = Tabs.GrabberTest.getFrontSpeed();
        GrabberSubsystem.getInstance().set(speed);

        Tabs.GrabberTest.displayFrontCurrent(GrabberSubsystem.getInstance().getGrabberCurrent());
    }
}

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
        GrabberSubsystem.getInstance().setLeft(speed);

        speed = Tabs.GrabberTest.getBackSpeed();
        GrabberSubsystem.getInstance().setRight(speed);

        Tabs.GrabberTest.displayFrontCurrent(GrabberSubsystem.getInstance().getTopGrabCurrent());
        Tabs.GrabberTest.displayBackCurrent(GrabberSubsystem.getInstance().getBotGrabCurrent());
    }
}

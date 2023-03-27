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
        if(Tabs.GrabberTest.moveFront() == 1){  //<-- Redundant and unnecessary
            double speed = Tabs.GrabberTest.getFrontSpeed();
            GrabberSubsystem.getInstance().setLeft(speed);
        }

        if(Tabs.GrabberTest.moveBack() == 1){
            double speed = Tabs.GrabberTest.getBackSpeed();
            GrabberSubsystem.getInstance().setRight(speed);
        }

        Tabs.GrabberTest.displayFrontCurrent(GrabberSubsystem.getInstance().getTopGrabCurrent());
        Tabs.GrabberTest.displayBackCurrent(GrabberSubsystem.getInstance().getBotGrabCurrent());
    }
}

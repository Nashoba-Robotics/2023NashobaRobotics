package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GrabberSubsystem;

public class ManualGrabberCommand extends CommandBase{
    double setPos3;
    public ManualGrabberCommand(){
        addRequirements(GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        GrabberSubsystem.getInstance().orient(0);
        setPos3 = 0;
    }

    @Override
    public void execute() {
        if(RobotContainer.operatorController.pov(0).getAsBoolean()) setPos3 -= 0.05;
        if(RobotContainer.operatorController.pov(180).getAsBoolean()) setPos3 += 0.05;
        
        SmartDashboard.putNumber("Set Pos Grab", setPos3);
        GrabberSubsystem.getInstance().orient(setPos3);
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().orient(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

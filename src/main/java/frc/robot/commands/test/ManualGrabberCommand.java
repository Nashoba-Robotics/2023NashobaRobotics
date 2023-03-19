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
        SmartDashboard.putNumber("Orient Speed", 0.05);
        GrabberSubsystem.getInstance().orient(0);
        setPos3 = 0;
    }

    @Override
    public void execute() {
        double speed = SmartDashboard.getNumber("Orient Speed", 0.05);
        if(RobotContainer.operatorController.pov(0).getAsBoolean()) setPos3 -= speed;
        if(RobotContainer.operatorController.pov(180).getAsBoolean()) setPos3 += speed;
        
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

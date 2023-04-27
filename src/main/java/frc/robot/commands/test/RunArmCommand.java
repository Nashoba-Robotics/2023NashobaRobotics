package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class RunArmCommand extends CommandBase {
    
    public RunArmCommand() {
        addRequirements(ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Extend Speed", 0);
        SmartDashboard.putNumber("Pivot Speed", 0);
    }

    @Override
    public void execute() {
        ArmSubsystem.getInstance().set(SmartDashboard.getNumber("Extend Speed", 0));
        ArmSubsystem.getInstance().setPivot(SmartDashboard.getNumber("Pivot Speed", 0));
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().set(0);
        ArmSubsystem.getInstance().setPivot(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

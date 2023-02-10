package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ZeroPivotCommand extends CommandBase{
    @Override
    public void initialize() {
        ArmSubsystem.getInstance().zeroPivot();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

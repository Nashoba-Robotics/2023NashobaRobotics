package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Tabs;
import frc.robot.subsystems.ArmSubsystem;

public class SetPivotOffsetCommand extends CommandBase {
    
    public SetPivotOffsetCommand() {
        
    }

    @Override
    public void initialize() {
        double offset = Tabs.Comp.getPivotOffset();
        ArmSubsystem.getInstance().addToAbsoluteOffset(offset);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

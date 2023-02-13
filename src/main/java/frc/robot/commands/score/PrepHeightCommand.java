package frc.robot.commands.score;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.Constants.Field.TargetLevel;


public class PrepHeightCommand extends CommandBase {
    private TargetLevel targetLevel;

    public PrepHeightCommand(TargetLevel targetLevel) {
        this.targetLevel = targetLevel;
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    public void initialize() {
        switch(targetLevel) {
            case HIGH: 
             GrabberSubsystem.getInstance().orientPos(-3);
             ArmSubsystem.getInstance().pivot(Constants.TAU / 6);
             break;
            case MID: 
             GrabberSubsystem.getInstance().orientPos(-3);
             ArmSubsystem.getInstance().pivot(65 * Constants.TAU / 360);
             break;
           // case LOW: 
           //  ArmSubsystem.getInstance().extend(0);
        }
       
    }

    public boolean isFinished() {
        return true;
    }
}

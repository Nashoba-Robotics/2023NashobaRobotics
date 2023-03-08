package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ControllerTestCommand extends CommandBase {
    
    @Override
    public void execute() {
        SmartDashboard.putBoolean("TopButton", RobotContainer.operatorController.pov(0).getAsBoolean());
        SmartDashboard.putBoolean("BottomButton", RobotContainer.operatorController.pov(180).getAsBoolean());
    }

}

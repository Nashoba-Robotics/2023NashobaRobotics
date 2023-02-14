package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Tabs;

public class ControllerTestCommand extends CommandBase {
    public ControllerTestCommand(){

    }
    
    @Override
    public void initialize() {
        Tabs.Test.displayTest(20);
    }
    @Override
    public void execute(){
        
    }
    public void end() {
        Tabs.Test.displayTest(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    // initilize
    // execute 
    // end
    // isFinished
}

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Tabs;
import frc.robot.subsystems.JoystickSubsytem;

public class ButtonTestCommand extends CommandBase{
    //Trying to get toggling Cardinal directions working with buttons

    private boolean pushed = false;
    @Override
    public void execute() {
        if(JoystickSubsytem.getInstance().getRightJoystick().button(2).getAsBoolean()){
            pushed = !pushed;
        }

        Tabs.Misc.displayPush(pushed);
    }
    
}

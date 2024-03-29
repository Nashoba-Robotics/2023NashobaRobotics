package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.lib.util.JoystickValues;

public class JoystickSubsystem extends SubsystemBase {
    
    private static JoystickSubsystem instance;

    private CommandJoystick rightJoystick;
    private CommandJoystick leftJoystick;

    private CommandJoystick operatorController;

    public JoystickSubsystem() {
        rightJoystick = new CommandJoystick(Constants.Joystick.RIGHT_JOYSTICK_PORT);
        leftJoystick = new CommandJoystick(Constants.Joystick.LEFT_JOYSTICK_PORT);
        operatorController = new CommandJoystick(Constants.Joystick.OPERATOR_PORT);
    }

    public static JoystickSubsystem getInstance() {
        if(instance == null) instance = new JoystickSubsystem();
        return instance;
    }

    public JoystickValues getRightJoystickValues() {
        return new JoystickValues(rightJoystick.getX(), -rightJoystick.getY()); 
        //Joysticks are weird in that moving the Joystick forward produces a negative value
    }

    public JoystickValues getLeftJoystickValues() {
        return new JoystickValues(leftJoystick.getX(), -leftJoystick.getY());
    }

    public CommandJoystick getRightJoystick() {
        return rightJoystick;
    }

    public CommandJoystick getLeftJoystick() {
        return leftJoystick;
    }

    public boolean getLeftButtonValue(int index) {
        return leftJoystick.button(index).getAsBoolean();
    }
    public boolean getRightButtonValue(int index){
        return rightJoystick.button(index).getAsBoolean();
    }

    public CommandJoystick getOperatorController(){
        return operatorController;
    }

    //Pushing up/down on right joystick
    public double getManualExtend(){
        double y = operatorController.getThrottle();
        y = Math.abs(y) < Constants.Joystick.MANUAL_EXTEND_DEADZONE ? 
        0 : 
        (y-Constants.Joystick.MANUAL_EXTEND_DEADZONE)/(1-Constants.Joystick.MANUAL_EXTEND_DEADZONE);
        // ^ Keeps the joystick input between 0-1 rather than 0.1-1
    
        return operatorController.getThrottle();
    }
    //Pushing left/right on left joystick
    public double getManualPivot(){
        double x = -operatorController.getX();  //Going down corresponds to positive direction
        x = Math.abs(x) < Constants.Joystick.MANUAL_PIVOT_DEADZONE ? 
        0 : 
        Math.signum(x)*(Math.abs(x)-Constants.Joystick.MANUAL_PIVOT_DEADZONE)/(1-Constants.Joystick.MANUAL_PIVOT_DEADZONE);

        if(x > 0) x *= Constants.Joystick.MANUAL_PIVOT_DOWN_SENSITIVITY;
        else x *= Constants.Joystick.MANUAL_PIVOT_UP_SENSITIVITY;

        return x;
    }
}

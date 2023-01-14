package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.util.JoystickValues;

public class JoystickSubsytem extends SubsystemBase {
    
    private static JoystickSubsytem instance;

    private Joystick rightJoystick;
    private Joystick leftJoystick;

    public JoystickSubsytem() {
        rightJoystick = new Joystick(Constants.Joystick.RIGHT_JOYSTICK_PORT);
        leftJoystick = new Joystick(Constants.Joystick.LEFT_JOYSTICK_PORT);
    }

    public static JoystickSubsytem getInstance() {
        if(instance == null) instance = new JoystickSubsytem();
        return instance;
    }

    public JoystickValues getRightJoystickValues() {
        return new JoystickValues(rightJoystick.getX(), -rightJoystick.getY());
    }

    public JoystickValues getLeftJoystickValues() {
        return new JoystickValues(leftJoystick.getX(), -leftJoystick.getY());
    }

    public Joystick getRightJoystick() {
        return rightJoystick;
    }

    public Joystick getLefJoystick() {
        return leftJoystick;
    }

}

package frc.robot.lib;

import frc.robot.Constants;

public class JoystickValues {

    public double x;
    public double y;

    public JoystickValues(double x, double y) {
        this.x = x;
        this.y = y;
    }

    //applies deadzone to object and returns object
    public JoystickValues applyDeadZone() {
        x = Math.abs(x) < Constants.Joystick.DEAD_ZONE ? 0 : x;
        y = Math.abs(y) < Constants.Joystick.DEAD_ZONE ? 0 : y;
        return this;
    }
    
}

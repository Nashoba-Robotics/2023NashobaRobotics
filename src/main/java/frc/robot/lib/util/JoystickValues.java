package frc.robot.lib.util;

import frc.robot.Constants;
import frc.robot.lib.math.Units;

public class JoystickValues {

    public double x;
    public double y;

    public JoystickValues(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public JoystickValues shape(double[] deadzones, double[] sensitivities) {
        shapeX(deadzones[0], sensitivities[0]);
        shapeY(deadzones[1], sensitivities[1]);
        return this;
    }

    public JoystickValues shape(double deadzone, double sensitivity) {
        shapeX(deadzone, sensitivity);
        shapeY(deadzone, sensitivity);
        return this;
    }

    public JoystickValues shapeX(double deadzone, double sensitivity) {
        applyDeadzoneX(deadzone);
        x = Math.pow(Math.abs(x), sensitivity) * Math.signum(x);
        return this;
    }

    public JoystickValues shapeY(double deadzone, double sensitivity) {
        applyDeadzoneY(deadzone);
        y = Math.pow(Math.abs(y), sensitivity) * Math.signum(y);
        return this;
    }

    public JoystickValues applyDeadzone(double deadzone) {
        applyDeadzone(deadzone, deadzone);
        return this;
    }

    public JoystickValues applyDeadzone(double deadzoneX, double deadzoneY) {
        applyDeadzoneX(deadzoneX);
        applyDeadzoneY(deadzoneY);
        return this;
    }

    public JoystickValues applyDeadzoneX(double deadzone) {
        if(Math.abs(x) < deadzone) {
            x = 0;
        } else {
            x = Math.signum(x) * (Math.abs(x) - deadzone) / (1 - deadzone);
        }
        return this;
    }

    public JoystickValues applyDeadzoneY(double deadzone) {
        if(Math.abs(y) < deadzone) {
            y = 0;
        } else {
            y = Math.signum(y) * (Math.abs(y) - deadzone) / (1 - deadzone);
        }
        return this;
    }

    public JoystickValues applyAngleDeadzone(double deadzone) {
        double angle = Math.atan2(y, x);

        if(Math.min(Math.abs(angle), Math.abs(Constants.TAU - angle)) < deadzone) y = 0;
        angle-=Constants.TAU/4;
        Units.constrainRad(angle);
        if(Math.min(Math.abs(angle), Math.abs(Constants.TAU - angle)) < deadzone) x = 0;
        
        return this;
    }

    public JoystickValues multiply(double multiplier) {
        x *= multiplier;
        y *= multiplier;
        return this;
    }
    
}

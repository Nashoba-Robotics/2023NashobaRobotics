package frc.robot.lib;

import frc.robot.Constants;

public class Units {
    // Converts from native units into degrees
    public static double NUToDeg(double angle){
        //Account for gear ratio
        angle /= Constants.SwerveBase.GEARRATIO;

        //Convert from native units into rotations;
        angle /= 2048;

        //Convert to degrees
        angle *= 360;

        return angle;
    }

    // Converts from degrees to native units
    public static double degToNU(double angle){
        //Convert from degrees into rotation
        angle /= 360;

        //Convert from rotations into Native Units
        angle *= 2048;

        //Account for gear ratio
        angle *= Constants.SwerveBase.GEARRATIO;

        return angle;
    }

    // Constrains the given angle to 180 DEGREES 
    public static double constrainDeg(double angle){
        angle %= 360;

        if(Math.abs(angle) <= 180){
            return angle;
        }

        if(angle > 0){
            return angle - 360;
        }
        else{
            return angle + 360;
        }
    }
}

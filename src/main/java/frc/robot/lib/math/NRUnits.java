package frc.robot.lib.math;

import frc.robot.Constants;

public class NRUnits {

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

    public static double constrainRad(double angle) {
        return constrainDeg(angle * 360 / Constants.TAU) * Constants.TAU / 360;
    }

    public static class Drive {
        // Converts from native units into degrees
        public static double NUToDeg(double angle){
            //Account for gear ratio
            angle /= Constants.Swerve.TURN_GEAR_RATIO;

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
            angle *= Constants.Swerve.TURN_GEAR_RATIO;

            return angle;
        }

        //Convert from Native Units to Radians
        public static double NUToRad(double angle){
            //Account for gear ratio
            angle /= Constants.Swerve.TURN_GEAR_RATIO;

            //Convert from native units into rotations;
            angle /= 2048;

            //Convert from rotations into radians
            angle *= 2*Math.PI;

            return angle;
        }

        //Convert from NU/100ms to Meters per Second
        public static double NUToMPS(double speed){
            //Convert from NU/100ms to NU/s
            speed *= 10;

            //Convert from NU/s to Rotations/s
            speed /= 2048;

            speed /= Constants.Swerve.MOVE_GEAR_RATIO;

            //Convert from Rotations/s to Meters/s
            speed *= Constants.TAU*Constants.Swerve.WHEELRADIUS;
            return speed;
        }

        public static double NUToM(double NU) {
            NU /= 2048;
    
            NU /= Constants.Swerve.MOVE_GEAR_RATIO;
    
            NU *= Constants.TAU*Constants.Swerve.WHEELRADIUS;
            return NU;
        }

        //Converts to NU/100ms then to 
        public static double toPercentOutput(double mps){
            //Convert to rotations per second
            double speed = mps/(Constants.Swerve.WHEELRADIUS*Constants.TAU);
            //Convert to NU per second
            speed *= 2048;
            speed *= Constants.Swerve.MOVE_GEAR_RATIO;
            //Convert to NU/100ms
            speed /= 10;

            return speed/Constants.Swerve.MAX_NATIVE_VELOCITY;
        }
    }

    public static class Extension {
        public static double mToNU(double m){
            m *= 1000;  //Convert m to mm
            m *= 58.4;  //Found through experimentation: 1mm = 58.4NU
    
            return m;
        }
    
        public static double NUToM(double pos){
            pos /= 58.4;
            pos /= 1000;
    
            return pos;
        }

        public static double mpsToNU(double mps){
            mps = mToNU(mps);
            mps /= 10;

            return mps;
        }

        public static double NUToMPS(double NU){
            NU *= 10;
            NU = NUToM(NU);

            return 10;
        }
    }
    
    public static class Pivot {
        public static double radToNU(double angle){
            //Convert angle into rotations
            angle /= Constants.TAU;
    
            //Convert from arm rotations into motor rotations
            angle *= Constants.Arm.PIVOT_GEARRATIO;
    
            //Convert rotations into NU
            angle *= 2048;
    
            return angle;
        }

        public static double degToNU(double angle){
            //Covnert deg to rad
            angle *= Constants.TAU/360;

            //Yes... I'm lazy
            return radToNU(angle);
        }

        public static double NUToRad(double pos){
            //Convert to rotations
            pos /= 2048;
    
            //Convert to rotation of the arm
            pos /= Constants.Arm.PIVOT_GEARRATIO;
    
            //Convert to radians
            pos *= Constants.TAU;
    
            return pos;
        }
    
    }    

    public static class Grabber {
        // Converts from degrees to rotations
        public static double radToNU(double angle){
            return angle / Constants.TAU * Constants.Grabber.GEAR_RATIO;
        }

        public static double NUtoRad(double rotations) {
            return rotations * Constants.TAU / Constants.Grabber.GEAR_RATIO;
        }
    }
}
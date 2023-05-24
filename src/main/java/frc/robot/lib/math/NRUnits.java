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

            //Convert to degrees
            angle *= 360;

            return angle;
        }

        // Converts from degrees to native units
        public static double degToNU(double angle){
            //Convert from degrees into rotation
            angle /= 360;

            //Account for gear ratio
            angle *= Constants.Swerve.TURN_GEAR_RATIO;

            return angle;
        }

        //Convert from Native Units to Radians
        public static double NUToRad(double angle){
            //Account for gear ratio
            angle /= Constants.Swerve.TURN_GEAR_RATIO;

            //Convert from rotations into radians
            angle *= Constants.TAU;

            return angle;
        }

        //Convert from NU/s to Meters per Second
        public static double NUToMPS(double speed){
            speed /= Constants.Swerve.MOVE_GEAR_RATIO;

            //Convert from Rotations/s to Meters/s
            speed *= Constants.TAU*Constants.Swerve.WHEELRADIUS;
            return speed;
        }

        public static double NUToM(double NU) {
            NU /= Constants.Swerve.MOVE_GEAR_RATIO;
    
            NU *= Constants.TAU*Constants.Swerve.WHEELRADIUS;
            return NU;
        }

        public static double MToNU(double m){
            m /= Constants.TAU*Constants.Swerve.WHEELRADIUS;
            m *= Constants.Swerve.MOVE_GEAR_RATIO;

            return m;
        }

        //Converts to NU/s then to 
        public static double toPercentOutput(double mps){
            //Convert to rotations per second
            double speed = mps/(Constants.Swerve.WHEELRADIUS*Constants.TAU);
            //Convert to NU per second
            speed *= Constants.Swerve.MOVE_GEAR_RATIO;

            return speed/Constants.Swerve.MAX_NATIVE_VELOCITY;
        }
    }

    public static class Extension {
        public static double mToNU(double m){
            m *= 1000;  //Convert to mm
            m /= Constants.Arm.MM_PER_NU;   //Convert to NU
    
            return m;
        }
    
        public static double NUToM(double pos){
            pos *= Constants.Arm.MM_PER_NU;    //Convert to mm
            pos /= 1000;    //conert to m
    
            return pos;
        }

        public static double NUtoMPS(double NU){
            //Converts NU/100ms to meters/s
            NU = NUToM(NU);
            NU *= 10;
    
            return NU;
        }

        public static double mpsToNU(double mps){
            mps = mToNU(mps);
            mps /= 10;
    
            return mps;
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

        public static double NUToRPS(double NU){
            NU *= 10;
            NU = NUToRad(NU);
            return NU;
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
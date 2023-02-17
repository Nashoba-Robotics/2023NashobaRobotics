package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//We're going to try to use Shuffleboard tabs for testing in order to keep the main tab more organized
//  For each Subsystem, we have a tab and GenericEntry for every value we want to read/set
//  We then have a display function to just display it to shuffleboard
//  We also have a get function that reads the value from shuffleboard to give the subsystem in a command
public final class Tabs {
    public static class Test{
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Test");

        private static GenericEntry dispTest = tab.add("Testing", 0).getEntry();
        private static GenericEntry getTest = tab.add("Get Test", 0).getEntry();

        public static void displayTest(int n){
            dispTest.setDouble(n);
        }

        public static double getTest(){
            return getTest.getDouble(0);
        }
    }
    public static class Intake{
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

        private static GenericEntry extendSpeed = tab.add("Arm Speed", 0).getEntry();

        //Arm Stuff
        private static GenericEntry setExtendNU = tab.add("Extend NU", 0).getEntry();
        private static GenericEntry setExtendM = tab.add("Extend Meters", 0).getEntry();
        private static GenericEntry dispExtendNU = tab.add("Actual Extend NU", 0).getEntry();
        private static GenericEntry dispExtendM = tab.add("Actual Extend Meters", 0).getEntry();
        private static GenericEntry extendStator = tab.add("Extend Stator", -1).getEntry();
        private static GenericEntry extendSupply = tab.add("Extend Supply", -1).getEntry();

        //Grabber Stuff
        private static GenericEntry topRollerSpeed = tab.add("Top Roller Speed", 0).getEntry();
        private static GenericEntry topRollerStator = tab.add("Top Roller Stator", 0).getEntry();
        private static GenericEntry bottomRollerSpeed = tab.add("Top Roller Speed", -1).getEntry();
        private static GenericEntry bottomRollerStator = tab.add("Top Roller Stator", -1).getEntry();

        //Arm & Grabber get() stuff
        private static GenericEntry pivotNU = tab.add("Pivot NU", 0).getEntry();
        private static GenericEntry pivotRad = tab.add("Pivot Angle", 0).getEntry();
        private static GenericEntry pivotStator = tab.add("Pivot Stator", -1).getEntry();
        private static GenericEntry pivotSupply = tab.add("Pivot Supply", -1).getEntry();

        private static GenericEntry setOrienterNU = tab.add("Wrist NU", 0).getEntry();
        private static GenericEntry setOrienterRad = tab.add("Wrist Angle", 0).getEntry();
        private static GenericEntry orienterStator = tab.add("Wrist Stator", -1).getEntry();
        private static GenericEntry dispOrienterNU = tab.add("Actual Wrist NU", 0).getEntry();
        private static GenericEntry dispOrienterRad = tab.add("Actual Wrist Angle", 0).getEntry();

        public static void displayExtendNU(double NU){
            dispExtendNU.setDouble(NU);
        }
        public static void displayExtendM(double m){
            dispExtendM.setDouble(m);
        }
        public static void displayExtendSpeed(double speed){
            extendSpeed.setDouble(speed);
        }
        public static void displayExtendCurrent(double stator, double supply){
            extendStator.setDouble(stator);
            extendSupply.setDouble(supply);
        }
        public static void displayPivotNU(double NU){
            pivotNU.setDouble(NU);
        }
        public static void displayPivotAngle(double angle){
            pivotRad.setDouble(angle);
        }
        public static void displayPivotCurrent(double stator, double supply){
            pivotStator.setDouble(stator);
            pivotSupply.setDouble(supply);
        }

        public static void displayTopStator(double stator){
            topRollerStator.setDouble(stator);
        }
        public static void displayBotStator(double stator){
            bottomRollerStator.setDouble(stator);
        }
        public static void displayOrientStator(double stator){
            orienterStator.setDouble(stator);
        }
        public static void displayOrienterNU(double NU){
            dispOrienterNU.setDouble(NU);
        }
        public static void displayOrienterAngle(double angle){
            dispOrienterRad.setDouble(angle);
        }

        public static double getExtendNU(){
            return setExtendNU.getDouble(0);
        }
        public static double getExtendM(){
            return setExtendM.getDouble(0);
        }
        public static double getPivotNU(){
            return pivotNU.getDouble(0);
        }
        public static double getPivotAngle(){
            return pivotRad.getDouble(0);
        }

        public static double getTopSpeed(){
            return topRollerSpeed.getDouble(0);
        }
        public static double getBotSpeed(){
            return bottomRollerSpeed.getDouble(0);
        }
        public static double getOrienterNU(){
            return setOrienterNU.getDouble(0);
        }
        public static double getOrienterRad(){
            return setOrienterRad.getDouble(0);
        }
    }
    
    public static class Auto{
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Auto");

        private static GenericEntry odometryX = tab.add("X", 0).getEntry();
        private static GenericEntry odometryY = tab.add("Y", 0).getEntry();
        private static GenericEntry odometryAngle = tab.add("Angle", 0).getEntry();
        private static GenericEntry gyroAngle = tab.add("Gyro Angle", 0).getEntry();

        public static void displayX(double x){
            odometryX.setDouble(x);
        }
        public static void displayY(double y){
            odometryY.setDouble(y);
        }
        public static void displayOdAngle(double a){
            odometryAngle.setDouble(a);
        }
        public static void displayGyro(double a){
            gyroAngle.setDouble(a);
        }
    }

    public static class Limelight{
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

        private static GenericEntry tx = tab.add("tx", 0).getEntry();
        private static GenericEntry ty = tab.add("ty", 0).getEntry();
        private static GenericEntry tv = tab.add("tv", 0).getEntry();

    }

    public static class Comp{
        //Camera Stream
        //Odometry
        //
    }
}

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//We're going to try to use Shuffleboard tabs for testing in order to keep the main tab more organized
//  For each Subsystem, we have a tab and GenericEntry for every value we want to read/set
//  We then have a display function to just display it to shuffleboard
//  We also have a get function that reads the value from shuffleboard to give the subsystem in a command
public final class Tabs {
    public static class Test{
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Test");

        public static void add(Sendable sendable){
            tab.add(sendable);
        }
        public static ComplexWidget add(String name, Sendable sendable){
            return tab.add(name, sendable);
        }
        public static ComplexWidget add(String name, Sendable sendable, int x, int y){
            return add(name, sendable).withPosition(x, y);
        }
        public static ComplexWidget add(String name, Sendable sendable, int x, int y, int w, int h){
            return add(name, sendable, x, y).withSize(w, h);
        }
        public static void add(String name, Object o){
            tab.add(name, o);
        }

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

        public static void add(Sendable sendable){
            tab.add(sendable);
        }
        public static ComplexWidget add(String name, Sendable sendable){
            return tab.add(name, sendable);
        }
        public static ComplexWidget add(String name, Sendable sendable, int x, int y){
            return add(name, sendable).withPosition(x, y);
        }
        public static ComplexWidget add(String name, Sendable sendable, int x, int y, int w, int h){
            return add(name, sendable, x, y).withSize(w, h);
        }
        public static void add(String name, Object o){
            tab.add(name, o);
        }

        private static ShuffleboardLayout sets = tab.getLayout("Sets", BuiltInLayouts.kGrid)
            .withPosition(0, 1)
            .withSize(2, 2);

        private static GenericEntry pivotDeg = sets.add("Pivot Degrees", 0)
            .withPosition(0, 0)
            .getEntry();
        private static GenericEntry setExtendNU = sets.add("Extend NU", 0)
            .withPosition(0, 1)
            .getEntry();
        private static GenericEntry rollerSpeed = sets.add("Roller Speed", 0)
            .withPosition(1, 0)
            .getEntry();
        private static GenericEntry setOrienterNU = sets.add("Wrist NU", 0)
            .withPosition(1, 1)
            .getEntry();
        
        private static ShuffleboardLayout values = tab.getLayout("Display Values", BuiltInLayouts.kGrid)
            .withPosition(2, 1)
            .withSize(5, 3);
        private static GenericEntry dispExtendNU = values.add("Actual Extend NU", 0)
            .withPosition(0, 0)
            .getEntry();
        private static GenericEntry extendStator = values.add("Extend Stator", -1)
            .withPosition(0, 1)
            .getEntry();
        private static GenericEntry extendSupply = values.add("Extend Supply", -1)
            .withPosition(0, 2)
            .getEntry();
        private static GenericEntry pivotAngle = values.add("Pivot Angle", 0)
            .withPosition(1, 0)
            .getEntry();
        private static GenericEntry pivotStator = values.add("Pivot Stator", -1)
            .withPosition(1, 1)
            .getEntry();
        private static GenericEntry pivotSupply = values.add("Pivot Supply", -1)
            .withPosition(1, 2)
            .getEntry();
        private static GenericEntry dispOrienterNU = values.add("Actual Wrist NU", 0)
            .withPosition(2, 0)
            .getEntry();
        private static GenericEntry orienterStator = values.add("Wrist Stator", -1)
            .withPosition(2, 1)
            .getEntry();
        private static GenericEntry topRollerStator = values.add("Top Roller Stator", 0)
            .withPosition(3, 0)
            .getEntry();
        private static GenericEntry bottomRollerStator = values.add("Bot Roller Stator", -1)
            .withPosition(3, 1)
            .getEntry();
        private static GenericEntry encoderAngle = values.add("Sensor Angle", 0)
            .withPosition(2, 2)
            .getEntry();
        private static GenericEntry actualMM = values.add("Extend MM", 0)
            .getEntry();
        private static GenericEntry pivotOutput = values.add("Pivot Output", 0)
            .getEntry();

        public static ShuffleboardLayout zeroes = tab.getLayout("Zeroes", BuiltInLayouts.kList)
            .withPosition(7, 0)
            .withSize(1, 3);


        public static void displayExtendNU(double NU){
            dispExtendNU.setDouble(NU);
        }
        public static void displayExtendCurrent(double stator, double supply){
            extendStator.setDouble(stator);
            extendSupply.setDouble(supply);
        }
        //Input in Radians
        public static void displayPivotAngle(double angle){
            pivotAngle.setDouble(angle * 360 / Constants.TAU);
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
        public static void displayEncoder(double angle){
            encoderAngle.setDouble(angle);
        }
        public static void displayPivotOutput(double out){
            pivotOutput.setDouble(out);
        }
        public static void displayMM(double NU){
            actualMM.setDouble(NU/58.4);
        }

        public static void resetExtendEntry(){
            setExtendNU.setDouble(0);
        }
        public static void resetPivotAngleEntry(){
            pivotDeg.setDouble(0);
        }
        public static void resetGrabSpeed(){
            rollerSpeed.setDouble(0);
        }
        public static void resetOrienter(){
            setOrienterNU.setDouble(0);
        }
        public static void resetAll(){
            resetExtendEntry();
            resetPivotAngleEntry();
            resetGrabSpeed();
            resetOrienter();
        }

        public static double getExtendNU(){
            return setExtendNU.getDouble(0);
        }

        //Input: Degrees    Output: Radians
        public static double getPivotAngle(){
            return pivotDeg.getDouble(0) * Constants.TAU/360;
        }

        public static double getGrabSpeed(){
            return rollerSpeed.getDouble(0);
        }
        public static double getOrienterNU(){
            return setOrienterNU.getDouble(0);
        }
    }
    
    public static class Auto{
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Auto");

        public static void add(Sendable sendable){
            tab.add(sendable);
        }
        public static ComplexWidget add(String name, Sendable sendable){
            return tab.add(name, sendable);
        }
        public static ComplexWidget add(String name, Sendable sendable, int x, int y){
            return add(name, sendable).withPosition(x, y);
        }
        public static ComplexWidget add(String name, Sendable sendable, int x, int y, int w, int h){
            return add(name, sendable, x, y).withSize(w, h);
        }
        public static void add(String name, Object o){
            tab.add(name, o);
        }

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

        public static void add(Sendable sendable){
            tab.add(sendable);
        }
        public static ComplexWidget add(String name, Sendable sendable){
            return tab.add(name, sendable);
        }
        public static ComplexWidget add(String name, Sendable sendable, int x, int y){
            return add(name, sendable).withPosition(x, y);
        }
        public static ComplexWidget add(String name, Sendable sendable, int x, int y, int w, int h){
            return add(name, sendable, x, y).withSize(w, h);
        }
        public static void add(String name, Object o){
            tab.add(name, o);
        }
    }

    public static class Comp{
        //Camera Stream
        //Odometry
        //
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Competition");
 
        public static void add(Sendable sendable){
            tab.add(sendable);
        }
        public static ComplexWidget add(String name, Sendable sendable){
            return tab.add(name, sendable);
        }
        public static ComplexWidget add(String name, Sendable sendable, int x, int y){
            return add(name, sendable).withPosition(x, y);
        }
        public static ComplexWidget add(String name, Sendable sendable, int x, int y, int w, int h){
            return add(name, sendable, x, y).withSize(w, h);
        }
        public static void add(String name, Object o){
            tab.add(name, o);
        }
    }
}

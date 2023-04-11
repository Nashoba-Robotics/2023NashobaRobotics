package frc.robot;

import java.io.FileNotFoundException;
import java.util.HashMap;

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
        private static HashMap<String, GenericEntry> widgets = new HashMap();

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

        //New idea to make Shuffleboard tabs easier to deal with
        public static void put(String name, double value){
            if(widgets.containsKey(name)){
                widgets.get(name).setDouble(value);
            }
            else{
                GenericEntry entry = tab.add(name, 0).getEntry();
                entry.setDouble(value);
                widgets.put(name, entry);
            }
        }
        public static double getDouble(String name){
            if(!widgets.containsKey(name)){
                return -6.9;    //<-- Arbitrary number
            }
            
            return widgets.get(name).getDouble(0);
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
        private static GenericEntry grabberStator = values.add("Top Roller Stator", 0)
            .withPosition(3, 0)
            .getEntry();
        private static GenericEntry encoderAngle = values.add("Sensor Angle", 0)
            .withPosition(2, 2)
            .getEntry();
        private static GenericEntry actualMM = values.add("Extend MM", 0)
            .getEntry();
        private static GenericEntry pivotOutput = values.add("Pivot Output", 0)
            .getEntry();

        public static ShuffleboardLayout zeroes = tab.getLayout("Zeroes", BuiltInLayouts.kList)
            .withPosition(7, 1)
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
            grabberStator.setDouble(stator);
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
        
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Competition");

        private static GenericEntry gyroAngle = tab.add("Gyro", 0).getEntry();
        
        /*
         * Falcon Angle         Target Angle        Pivot Error
         * Encoder Angle        Target Angle        Encoder Error
         * Extend NU            Target Extend       Extend Error
         * Wrist NU             Target Wrist        Wrist Error
         */
        private static ShuffleboardLayout error = tab.getLayout("Errors", BuiltInLayouts.kGrid)
            .withSize(3, 4);
        
        //Actual
        private static GenericEntry pivotAngle = error.add("Actual Pivot", 0)
            .withPosition(0, 0)    
            .getEntry();
        private static GenericEntry encoderAngle = error.add("Encoder Angle", 0)
            .withPosition(0, 1)
            .getEntry();
        private static GenericEntry extendNU = error.add("Extend NU", 0)
            .withPosition(0, 2)
            .getEntry();
        private static GenericEntry wristNU = error.add("Wrist NU", 0)
            .withPosition(0, 3)
            .getEntry();

        //Target
        private static GenericEntry targetPivot = error.add("Target Pivot", 0)  
            .withPosition(1, 0)
            .getEntry();
        private static GenericEntry targetExtend = error.add("Target Extend", 0)
            .withPosition(1, 2)    
            .getEntry();
        private static GenericEntry targetWrist = error.add("Target Wrist", 0)
            .withPosition(1, 3)
            .getEntry();

        //Error
        private static GenericEntry pivotError = error.add("Pivot Error", 0)
            .withPosition(2, 0)
            .getEntry();
        private static GenericEntry encoderError = error.add("Encoder Error", 0)
            .withPosition(2, 1)
            .getEntry();
        private static GenericEntry extendError = error.add("Extend Error", 0)
            .withPosition(2, 2)
            .getEntry();
        private static GenericEntry wristError = error.add("Wrist Error", 0)
            .withPosition(2, 3)
            .getEntry();

        private static GenericEntry setPivotOffset = tab.add("Set Pivot Offset", Constants.Arm.ENCODER_OFFSET).getEntry();

        private static GenericEntry grabberRunning = tab.add("Grabber On", false).getEntry();

        public static void displayGrabberRunning(boolean running) {
            grabberRunning.setBoolean(running);
        }
 
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

        public static double getPivotOffset(){
            return setPivotOffset.getDouble(Constants.Arm.ENCODER_OFFSET);
        }

        public static void displayGyro(double angle){
            gyroAngle.setDouble(angle);
        }

        public static void displayPivotAngle(double rad){
            rad *= 360/Constants.TAU;

            pivotAngle.setDouble(rad);
            
            double error = targetPivot.getDouble(0) - pivotAngle.getDouble(0);
            pivotError.setDouble(error);
        }
        public static void displayEncoderAngle(double deg){
            encoderAngle.setDouble(deg);

            double error = targetPivot.getDouble(0) - encoderAngle.getDouble(0);
            encoderError.setDouble(error);
        }
        public static void displayExtendNU(double NU){
            extendNU.setDouble(NU);

            double error = targetExtend.getDouble(0) - extendNU.getDouble(0);
            extendError.setDouble(error);
        }
        public static void displayWristNU(double NU){
            wristNU.setDouble(NU);

            double error = targetWrist.getDouble(0) - wristNU.getDouble(0);
            wristError.setDouble(error);
        }

        public static void setPivotTarget(double target){
            targetPivot.setDouble(target * 360/Constants.TAU);
        }
        public static void setExtendTarget(double target){
            targetExtend.setDouble(target);
        }
        public static void setWristTarget(double target){
            targetWrist.setDouble(target);
        }
    }

    public static class Misc{
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Miscelaneous");

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

        //Temporary button stuff
        private static GenericEntry buttonPushed = tab.add("Pushed", false).getEntry();

        public static void displayPush(boolean pushed){
            buttonPushed.setBoolean(pushed);
        }

    }

    public static class DriveTest{
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Drive Test");

        private static GenericEntry move = tab.add("Move", 0).getEntry();

        private static GenericEntry driveSpeed = tab.add("Drive Speed", 0).getEntry();
        private static GenericEntry actualSpeed = tab.add("Actual Speed", 0).getEntry();

        private static GenericEntry pivotAngle = tab.add("Pivot Angle Deg", 0).getEntry();
        private static GenericEntry actualPivotAngle = tab.add("Actual Pivot Angle", 0).getEntry();

        private static GenericEntry grabberAngle = tab.add("Grabber NU", 0).getEntry();

        private static GenericEntry grabberCurrent = tab.add("Grabber Current", 0).getEntry();
        private static GenericEntry grabberSpeed = tab.add("Grabber Speed", 0.7).getEntry();

        public static void displayActualSpeed(double speed){
            actualSpeed.setDouble(speed);
        }
        public static void displayActualAngle(double angle) {
            actualPivotAngle.setDouble(angle);
        }
        public static void displayMove(){
            move.setDouble(0);
        }

        public static void displayGrabberCurrent(double current) {
            grabberCurrent.setDouble(current);
        }

        public static double getGrabberSpeed() {
            return grabberSpeed.getDouble(0.7);
        }

        public static double getDriveSpeed(){
            return driveSpeed.getDouble(0);
        }
        public static double getPivotAngle(){
            return pivotAngle.getDouble(0);
        }
        public static double grabberAngle(){
            return grabberAngle.getDouble(0);
        }
        public static boolean move(){
            return move.getDouble(0) == 1;
        }
    }

    public static class GrabberTest{
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Grabber Test");

        private static GenericEntry frontRollerSpeed = tab.add("Front Speed", 0).getEntry();
        private static GenericEntry backRollerSpeed = tab.add("Back Speed", 0).getEntry();

        private static GenericEntry runFront = tab.add("Run Front", 0).getEntry();
        private static GenericEntry runBack = tab.add("Run Back", 0).getEntry();

        private static GenericEntry frontCurrent = tab.add("Front Curr", 0).getEntry();
        private static GenericEntry backCurrent = tab.add("Back Curr", 0).getEntry();

        public static double getFrontSpeed(){
            return frontRollerSpeed.getDouble(0);
        }
        public static double getBackSpeed(){
            return backRollerSpeed.getDouble(0);
        }

        public static double moveFront(){
            return runFront.getDouble(0);
        }
        public static double moveBack(){
            return runBack.getDouble(0);
        }

        public static void displayFrontCurrent(double curr){
            frontCurrent.setDouble(curr);
        }
        public static void displayBackCurrent(double curr){
            backCurrent.setDouble(curr);
        }
    }
}

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//Reflective Tape sorted by highest

//In robot, the LEDs on the limelight should be off when the robot first turns on
//During autonomous and teleop, the limelight LEDs will follow the pipeline
//When the robot is disabled, the LEDs turn off
//TODO: Get constants for the height and angle of the limelight on robot
public class LimelightSubsystem extends SubsystemBase{
    private static LimelightSubsystem singleton;
    NetworkTable nt;

    //General Limelight Entries
    NetworkTableEntry tvEntry;
    NetworkTableEntry txEntry;
    NetworkTableEntry tyEntry;

    NetworkTableEntry pipeline;
    NetworkTableEntry ledMode;

    //April Tag Entries
    NetworkTableEntry tidEntry;
    
    //Classifier/Detector Entries
    NetworkTableEntry tclassEntry;


    boolean isTarget;
    double[] robotPos;
    double tx;
    double ty;
    double tagID;

    public enum TargetType{
        APRIL_TAG,
        REFLECTIVE_TAPE
    }

    private LimelightSubsystem(){
        nt = NetworkTableInstance.getDefault().getTable("limelight");

        txEntry = nt.getEntry("tx");
        tyEntry = nt.getEntry("ty");
        tvEntry = nt.getEntry("tv");
        tidEntry = nt.getEntry("tid");

        pipeline = nt.getEntry("pipeline");
    }

    public static LimelightSubsystem getInstance(){
        if(singleton == null) singleton = new LimelightSubsystem();
        return singleton;
    }

    @Override
    public void periodic() {
        isTarget = tvEntry.getDouble(0) == 1;
        if(isTarget){   //Only update information if we see a target
            if(getPipeline() == Constants.Limelight.APRIL_TAG_PIPELINE){
                if(DriverStation.getAlliance() == Alliance.Red)
                    robotPos = nt.getEntry("botpose_wpired").getDoubleArray(new double[6]);
                else if(DriverStation.getAlliance() == Alliance.Blue)
                    robotPos = nt.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
                else
                    robotPos = nt.getEntry("botpose").getDoubleArray(new double[6]);
                tagID = tidEntry.getDouble(-1);
            }
            tx = txEntry.getDouble(0);
            ty = tyEntry.getDouble(0);
        }
    }

    // Gets the 2d position of the robot using the april tag it sees
    public Pose2d getRobotPose(){
        int pipeline = getPipeline();
        setPipeline(Constants.Limelight.APRIL_TAG_PIPELINE);
        Pose2d pose = new Pose2d(robotPos[0], robotPos[1], new Rotation2d(robotPos[3], robotPos[4]));
        setPipeline(pipeline);
        return pose;
    }

    // Returns how far away the target is from the crosshairs(center) horizontally
    public double getTX(){
        return tx;
    }

    // Returns how far away the target is from the crosshairs(center) vertically
    public double getTY(){
        return ty;
    }

    // Returns whether or not the camera can see a target (For Reflective Tape, April Tag, and Detector)
    public boolean isTarget(){
        return isTarget;
    }

    // Returns the current pipeline as an int
    public int getPipeline(){
        return (int)pipeline.getInteger(-1);
    }

    // I couldn't figure out how to change the Point of Interest through code, so we'll probably have to change pipelines for that
    // (We can set the node to be or point of interest to make targeting easier)
    // Switches pipeline taking input as the integer version of the pipeline
    public void setPipeline(int index){
        pipeline.setInteger(index);
    }

    //0 = follow pipeline, 1 = force off, 2 = force blink, 3 = force on
    public void setLEDMode(int mode){
        ledMode.setNumber(mode);
    }

    public void on(){
        setLEDMode(3);
    }

    public void off(){
        setLEDMode(1);
    }

    public void defaultLED(){
        setLEDMode(0);
    }
}
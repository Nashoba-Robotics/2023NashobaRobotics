package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase{
    private static LimelightSubsystem singleton;
    NetworkTable nt;
    NetworkTableEntry botpose;
    NetworkTableEntry txEntry;
    NetworkTableEntry tyEntry;
    NetworkTableEntry tvEntry;
    NetworkTableEntry tidEntry;

    NetworkTableEntry pipeline;

    boolean isTarget;
    double[] robotPos;
    double tx;
    double ty;
    double tagID;


    private LimelightSubsystem(){
        nt = NetworkTableInstance.getDefault().getTable("limelight");
        botpose = nt.getEntry("botpose");
        txEntry = nt.getEntry("tx");
        tyEntry = nt.getEntry("ty");
        tvEntry = nt.getEntry("tv");
        tidEntry = nt.getEntry("tid");

        pipeline = nt.getEntry("pipeline");
    }

    @Override
    public void periodic() {
        isTarget = tvEntry.getBoolean(false);
        if(isTarget){   //Only update information if we see a target
            robotPos = botpose.getDoubleArray(new double[6]);
            tx = txEntry.getDouble(0);
            ty = tyEntry.getDouble(0);
            tagID = tidEntry.getDouble(-1);
        }
    }

    public static LimelightSubsystem getInstance(){
        if(singleton == null) singleton = new LimelightSubsystem();
        return singleton;
    }

    // Gets the 2d position of the robot using the april tag it sees
    public Pose2d getRobotPose(){
        return new Pose2d(robotPos[0], robotPos[1], new Rotation2d(robotPos[3], robotPos[4]));
    }

    public double getTX(){
        return tx;
    }

    public double getTY(){
        return ty;
    }

    public double isTarget(){
        return isTarget();
    }

    //Don't know if this is correct
    public int getPipeline(){
        return pipeline.getHandle();
    }

    // I couldn't figure out how to change the Point of Interest through code, so we'll probably have to change pipelines for that
    // (We can set the node to be or point of interest to make targeting easier)
    public void setPipeline(int index){
        pipeline.setInteger(index);
    }
}

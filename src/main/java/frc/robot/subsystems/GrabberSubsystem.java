package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase{
    /*private CANSparkMax leftGrabber;    //I don't know if this is actually how to make a SparkMax instance
    private CANSparkMax rightGrabber;

    private CANSparkMax wrist;

    public GrabberSubsystem(){
        leftGrabber = new CANSparkMax(Constants.Grabber.LEFT_GRABBER_PORT, MotorType.kBrushed); //CHANGE: I don't know if the motors will be brushed or brushless
        rightGrabber = new CANSparkMax(Constants.Grabber.RIGHT_GRABBER_PORT, MotorType.kBrushed);

        wrist = new CANSparkMax(Constants.Grabber.WRIST_PORT, MotorType.kBrushed);
    }

    private static GrabberSubsystem singleton;
    public static GrabberSubsystem getInstance(){
        if(singleton == null) singleton = new GrabberSubsystem();
        return singleton;
    }

    //Gets the game piece
    public void intake(){

    }

    //Releases the game piece to score
    public void score(){

    }

    //Turns the wrist to specified angle
    public void orient(double angle){

    }
     */
}

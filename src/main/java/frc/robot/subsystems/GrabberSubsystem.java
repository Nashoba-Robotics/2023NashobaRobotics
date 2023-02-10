package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.Units;

public class GrabberSubsystem extends SubsystemBase{
    private CANSparkMax grabber1;    //I don't know if this is actually how to make a SparkMax instance
    private CANSparkMax grabber2;

    private CANSparkMax orienter;
    private SparkMaxPIDController orienterController;

    private double currentRotation = 0;

    public GrabberSubsystem(){
        grabber1 = new CANSparkMax(Constants.Grabber.LEFT_GRABBER_PORT, MotorType.kBrushed); //CHANGE: I don't know if the motors will be brushed or brushless
        grabber2 = new CANSparkMax(Constants.Grabber.RIGHT_GRABBER_PORT, MotorType.kBrushed);

        orienter = new CANSparkMax(Constants.Grabber.WRIST_PORT, MotorType.kBrushed);

        grabber1.setIdleMode(IdleMode.kBrake);
        grabber2.setIdleMode(IdleMode.kBrake);
        grabber2.follow(grabber1);

        currentRotation = orienter.getEncoder().getPosition();

        orienterController = orienter.getPIDController();
        orienterController.setFF(Constants.Grabber.ORIENTER_KF);
        orienterController.setP(Constants.Grabber.ORIENTER_KP);
        orienterController.setI(Constants.Grabber.ORIENTER_KI);
        orienterController.setD(Constants.Grabber.ORIENTER_KD);
        orienterController.setOutputRange(-Constants.Grabber.TURN_SPEED, Constants.Grabber.TURN_SPEED);
    }

    private static GrabberSubsystem singleton;
    public static GrabberSubsystem getInstance(){
        if(singleton == null) singleton = new GrabberSubsystem();
        return singleton;
    }

    //Gets the game piece
    public void intake(){
        grabber1.set(Constants.Grabber.INTAKE_SPEED);
    }

    //Releases the game piece to score
    public void score(){
        grabber1.set(-Constants.Grabber.INTAKE_SPEED);
    }

    //Turns the orienter to specified angle
    public void orient(double angle){
        currentRotation = Units.Grabber.degToNU(angle);
    }

    @Override
    public void periodic() {
        orienterController.setReference(currentRotation, ControlType.kPosition);
    }
}

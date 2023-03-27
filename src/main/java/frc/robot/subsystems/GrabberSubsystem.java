package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LogManager;
import frc.robot.Tabs;
import frc.robot.lib.math.NRUnits;

public class GrabberSubsystem extends SubsystemBase{
    private CANSparkMax grabber1;    //I don't know if this is actually how to make a SparkMax instance
    private CANSparkMax grabber2;

    private CANSparkMax orienter;
    private SparkMaxPIDController orienterController;
    private RelativeEncoder orientEncoder;

    //LIDAR
    private SerialPort lidar = new SerialPort(115200, SerialPort.Port.kUSB1);
    private byte[] recieve = new byte[18];  //There are 9 bytes of data that are useful to us
    /*  *There is a weird problem where the lidar does not read the bytes in correct order, so we're going to be reading 18 bytes instead of 9
     *  2 Headers of value 89
     *  2 Distance values after that 
     *  2 Strength values
     *  2 Temperature values
     */

    public GrabberSubsystem(){
        grabber1 = new CANSparkMax(Constants.Grabber.LEFT_GRABBER_PORT, MotorType.kBrushless); //CHANGE: I don't know if the motors will be brushed or brushless
        grabber2 = new CANSparkMax(Constants.Grabber.RIGHT_GRABBER_PORT, MotorType.kBrushless);


        orienter = new CANSparkMax(Constants.Grabber.WRIST_PORT, MotorType.kBrushless);
        orientEncoder = orienter.getEncoder();

        grabber1.setIdleMode(IdleMode.kBrake);
        grabber2.setIdleMode(IdleMode.kBrake);

        grabber1.setSmartCurrentLimit(30);
        grabber2.setSmartCurrentLimit(30);

        orienterController = orienter.getPIDController();
        orienterController.setFF(Constants.Grabber.ORIENTER_KF);
        orienterController.setP(Constants.Grabber.ORIENTER_KP);
        orienterController.setI(Constants.Grabber.ORIENTER_KI);
        orienterController.setD(Constants.Grabber.ORIENTER_KD);
        orienterController.setOutputRange(-Constants.Grabber.MAX_TURN_SPEED, Constants.Grabber.MAX_TURN_SPEED);
    }

    private static GrabberSubsystem singleton;
    public static GrabberSubsystem getInstance(){
        if(singleton == null) singleton = new GrabberSubsystem();
        return singleton;
    }

    //Gets the game piece
    public void intake(){
        set(Constants.Grabber.INTAKE_SPEED);
    }

    //Releases the game piece to score
    public void score(){
        grabber1.set(Constants.Grabber.SCORE_SPEED);
    }

    public void setLeft(double speed){
        grabber1.set(speed);
    }

    public void setRight(double speed){
        grabber2.set(speed);
    }

    public void set(double leftSpeed, double rightSpeed){
        setLeft(leftSpeed);
        setRight(rightSpeed);
    }

    public void set(double speed){
        set(speed, speed);
    }

    //Turns the orienter to specified angle in radians
    public void orient(double angle){
        orienterController.setReference(NRUnits.Grabber.radToNU(angle), ControlType.kPosition);
    }

    public void orientPos(double NU) {
        orienterController.setReference(NU, ControlType.kPosition);
    }

    public void setOrientSpeed(double speed){
        orienter.set(speed);
    }

    public double getOrientPos(){
        return orientEncoder.getPosition();
    }

    //Reads the angle in radians of the orienter
    public double getOrientation(){
        return NRUnits.Grabber.NUtoRad(orientEncoder.getPosition());
    }

    public double getPosition() {
        return orientEncoder.getPosition();
    }

    public void zeroWrist() {
        orientEncoder.setPosition(0);
    }

    public double getCurrent() {
        // Average
        return (getTopGrabCurrent()+getBotGrabCurrent()) / 2;
    }

    public double getTopGrabCurrent(){
        return grabber1.getOutputCurrent();
    }
    public double getBotGrabCurrent(){
        return grabber2.getOutputCurrent();
    }

    public void stop() {
        grabber1.set(0);
        orienter.set(0);
    }

    public void setCurrentLimit(int limit) {
        grabber1.setSmartCurrentLimit(limit);
        grabber2.setSmartCurrentLimit(limit);
    }

    public double getLidarDist(){
        recieve = lidar.read(18);

        for(int i = 0; i < recieve.length; i++){
            if(recieve[i] == 89){
                if(recieve[i+1] == 89)
                    return recieve[i+2];    //The data will look something along the lines of _ 89 89 x _ _
                else
                    return recieve[i+1];
            }
        }
        return -1.2;    //Random arbitrary value that tells us it's not working
    }

    @Override
    public void periodic() {
        if(Constants.Logging.GRABBER) {
            //Grabber1
            LogManager.appendToLog(grabber1.getEncoder().getVelocity(), "Grabber:/Grabber1/Velocity");
            LogManager.appendToLog(grabber1.getBusVoltage() * grabber1.getAppliedOutput(), "Grabber:/Grabber1/Voltage");

            //Grabber2
            LogManager.appendToLog(grabber2.getEncoder().getVelocity(), "Grabber:/Grabber2/Velocity");
            LogManager.appendToLog(grabber2.getBusVoltage() * grabber2.getAppliedOutput(), "Grabber:/Grabber2/Voltage");

            //Orienter
            LogManager.appendToLog(orientEncoder.getPosition(), "Grabber:/Orienter/Position");
        }

        SmartDashboard.putNumber("grabber1", grabber1.getOutputCurrent());
        SmartDashboard.putNumber("grabber2", grabber2.getOutputCurrent());

        Tabs.Comp.displayWristNU(getOrientPos());
    }
}

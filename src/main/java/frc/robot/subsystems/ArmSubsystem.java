package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;
import frc.robot.lib.math.Units;

public class ArmSubsystem{
    private TalonFX tromboneSlide;  //Controls the extension/retraction of the arm
    private TalonFX pivot1, pivot2; //Control the pivoting of the entire arm

    public ArmSubsystem(){
        tromboneSlide = new TalonFX(Constants.Arm.ARM_PORT);

        pivot1 = new TalonFX(Constants.Arm.PIVOT_PORT_1);
        pivot2 = new TalonFX(Constants.Arm.PIVOT_PORT_2);

        config();
    }

    private static ArmSubsystem singleton;
    public static ArmSubsystem getInstance(){
        if(singleton == null) singleton = new ArmSubsystem();
        return singleton;
    }

    public void config(){
        tromboneSlide.configFactoryDefault();
        tromboneSlide.setNeutralMode(NeutralMode.Brake);
        tromboneSlide.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        tromboneSlide.config_kF(0, Constants.Arm.ARM_KF);
        tromboneSlide.config_kP(0, Constants.Arm.ARM_KP);
        tromboneSlide.config_kI(0, Constants.Arm.ARM_KI);
        tromboneSlide.config_kD(0, Constants.Arm.ARM_KD);

        tromboneSlide.configMotionCruiseVelocity(Constants.Arm.ARM_CRUISE_VELOCITY);
        tromboneSlide.configMotionAcceleration(Constants.Arm.ARM_ACCELERATION);

        pivot1.configFactoryDefault();
        pivot1.setNeutralMode(NeutralMode.Brake);
        pivot1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        pivot1.config_kF(0, Constants.Arm.PIVOT_KF_1);
        pivot1.config_kP(0, Constants.Arm.PIVOT_KP_1);
        pivot1.config_kI(0, Constants.Arm.PIVOT_KI_1);
        pivot1.config_kD(0, Constants.Arm.PIVOT_KD_1);

        pivot2.configFactoryDefault();
        pivot2.setNeutralMode(NeutralMode.Brake);
        pivot2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        pivot2.config_kF(0, Constants.Arm.PIVOT_KF_2);
        pivot2.config_kP(0, Constants.Arm.PIVOT_KP_2);
        pivot2.config_kI(0, Constants.Arm.PIVOT_KI_2);
        pivot2.config_kD(0, Constants.Arm.PIVOT_KD_2);
    }

    public void zeroArm(){
        tromboneSlide.setSelectedSensorPosition(0);
    }

    public void zeroPivot1(){
        pivot1.setSelectedSensorPosition(0);
    }

    public void zeroPivot2(){
        pivot2.setSelectedSensorPosition(0);
    }

    public void zeroPivot(){
        zeroPivot1();
        zeroPivot2();
    }

    //Returns arm to upright position
    public void reset(){

    }

    //Extends arm to specified position in meters
    public void extend(double pos){
        tromboneSlide.set(ControlMode.MotionMagic, Units.Arm.mToNU(pos));
    }

    //Pivots arm to specified angle (Where to define 0? Radians or degrees?)
    public void pivot(double angle){
        
    }

    //Returns the angle of the arm
    public double getAngle(){
        return 0;
    }

    //Returns the extension of the arm in meters
    public double getLength(){
        double pos = tromboneSlide.getSelectedSensorPosition();

        return Units.Arm.NUToM(pos);
    }

}
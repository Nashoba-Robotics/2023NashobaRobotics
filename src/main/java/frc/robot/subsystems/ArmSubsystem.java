package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

public class ArmSubsystem{
    private TalonFX extender;
    private TalonFX pivot1, pivot2;

    public ArmSubsystem(){
        extender = new TalonFX(Constants.Arm.EXTEND_PORT);

        pivot1 = new TalonFX(Constants.Arm.PIVOT_PORT_1);
        pivot2 = new TalonFX(Constants.Arm.PIVOT_PORT_2);
    }

    private static ArmSubsystem singleton;
    public static ArmSubsystem getInstance(){
        if(singleton == null) singleton = new ArmSubsystem();
        return singleton;
    }

    //Extends arm to specified position (What units?)
    public void extend(double pos){

    }

    //Pivots arm to specified angle (Where to define 0? Radians or degrees?)
    public void pivot(double angle){

    }

}
package frc.robot.commands.test;

import java.lang.constant.Constable;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class PrepTestCommand extends CommandBase{
    double l0 = 0.690;

    public PrepTestCommand(){
        addRequirements(ArmSubsystem.getInstance());
    }

    public double NUtoMPS(double NU){
        //Converts NU/100ms to meters/s
        NU *= 10;
        NU /= 58.4;
        NU /= 1000;

        return NU;
    }

    public double mpsToNU(double mps){
        mps *= 1000;
        mps *= 58.4;
        mps /= 10;

        return mps;
    }
    @Override
    public void initialize() {
        ArmSubsystem.getInstance().pivot(Constants.Arm.HIGH_ANGLE);
        ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(50_000);
        ArmSubsystem.getInstance().setPivotAcceleration(55_000);

        ArmSubsystem.getInstance().setExtendAcceleration(50_000);
    }

    @Override
    public void execute() {
        //l0 = 0.690m
        double pivotAngle = ArmSubsystem.getInstance().getAngle();
        double pivotSpeed = ArmSubsystem.getInstance().getPivotSpeed();
        pivotSpeed = NUtoMPS(pivotSpeed);

        double extendSpeed = l0 * Math.tan(pivotAngle)/Math.cos(pivotAngle)*pivotSpeed;
        extendSpeed = mpsToNU(extendSpeed);

        ArmSubsystem.getInstance().setExtendCruiseVelocity(extendSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        ArmSubsystem.getInstance().setDefaultAcceleration();

        ArmSubsystem.getInstance().setExtendCruiseVelocity(Constants.Arm.ARM_CRUISE_VELOCITY);
        ArmSubsystem.getInstance().setExtendAcceleration(15_000);

        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

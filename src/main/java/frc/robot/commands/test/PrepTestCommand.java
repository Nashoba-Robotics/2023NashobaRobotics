package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;
import frc.robot.subsystems.ArmSubsystem;

public class PrepTestCommand extends CommandBase{

    public PrepTestCommand(){
        addRequirements(ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().setPivotCruiseVelocity(50_000);
        ArmSubsystem.getInstance().setPivotAcceleration(55_000);

        ArmSubsystem.getInstance().setExtendAcceleration(50_000);

        //Tell the arm the target angle and extension
        ArmSubsystem.getInstance().pivot(Constants.Arm.HIGH_ANGLE);
        ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU);
    }

    @Override
    public void execute() {
        //l0 = 0.690m
        double pivotAngle = ArmSubsystem.getInstance().getAngle();
        double pivotSpeed = ArmSubsystem.getInstance().getPivotSpeed();
        pivotSpeed = NRUnits.Arm.NUtoMPS(pivotSpeed);

        double extendSpeed = Constants.Arm.l0 * Math.tan(pivotAngle)/Math.cos(pivotAngle)*pivotSpeed;
        extendSpeed = NRUnits.Arm.mpsToNU(extendSpeed);

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

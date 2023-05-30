package frc.robot.commands.test;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.NewTabs;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSpeedTestCommand extends CommandBase{
    private String tabTitle = "Arm Speed Test";

    public ArmSpeedTestCommand(){
        addRequirements(ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        if(Robot.state == RobotState.OK && ArmSubsystem.getInstance().pivotStopped())ArmSubsystem.getInstance().resetPivotNU();
    }

    @Override
    public void execute() {
        ArmSubsystem.getInstance().setPivotCruiseVelocity(100);
        ArmSubsystem.getInstance().setPivotAcceleration(300);
        ArmSubsystem.getInstance().setExtendCruiseVelocity(100);
        ArmSubsystem.getInstance().setExtendAcceleration(150);
        
        ArmSubsystem.getInstance().pivot(Constants.Arm.HIGH_FRONT_ANGLE);
        ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU);

        NewTabs.putDouble(tabTitle, "Pivot Deg", ArmSubsystem.getInstance().getPivotDeg());
        NewTabs.putDouble(tabTitle, "CANcoder Deg", ArmSubsystem.getInstance().getEncoderDeg());
        NewTabs.putDouble(tabTitle, "Extend NU", ArmSubsystem.getInstance().getExtendNU());
    }

    @Override
    public void end(boolean interrupted) {
        double pivotAccel = NewTabs.getDouble(tabTitle, "Pivot Accel", 40);
        double pivotCruise = NewTabs.getDouble(tabTitle, "Pivot Cruise", 100);

        double extendAccel = NewTabs.getDouble(tabTitle, "Extend Accel", 50);
        double extendCruise = NewTabs.getDouble(tabTitle, "Extend Cruise", 100);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(pivotCruise);
        ArmSubsystem.getInstance().setPivotAcceleration(pivotAccel);
        ArmSubsystem.getInstance().setExtendCruiseVelocity(extendCruise);
        ArmSubsystem.getInstance().setExtendAcceleration(extendAccel);

        ArmSubsystem.getInstance().pivot(Constants.Arm.PREP_ANGLE);
        ArmSubsystem.getInstance().extendNU(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

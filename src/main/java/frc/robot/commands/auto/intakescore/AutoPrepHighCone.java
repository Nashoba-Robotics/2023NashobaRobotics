package frc.robot.commands.auto.intakescore;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoPrepHighCone extends CommandBase{
    @Override
    public void initialize() {
        GrabberSubsystem.getInstance().orientPos(Constants.Grabber.PREP_CONE_FRONT_NU);
        ArmSubsystem.getInstance().pivot(Constants.Arm.HIGH_FRONT_ANGLE);

        ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU-2100);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(ArmSubsystem.getInstance().getExtendNU()-Constants.Arm.HIGH_EXTEND_NU) < 1000
        &&     Math.abs(ArmSubsystem.getInstance().getPivotAngle(1)-Constants.Arm.HIGH_FRONT_ANGLE) < 2;
    }
}

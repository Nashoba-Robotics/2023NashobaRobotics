package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

//Cones ONLY!!
public class DoubleStationIntakeCommand extends CommandBase{
    public DoubleStationIntakeCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        ArmSubsystem.getInstance().setDefaultAcceleration();

        ArmSubsystem.getInstance().pivot(Constants.Arm.DOUBLE_STATION_ANGLE);
        ArmSubsystem.getInstance().extendNU(Constants.Arm.DOUBLE_STATION_EXTEND_NU);

        GrabberSubsystem.getInstance().orientPos(Constants.Grabber.DOUBLE_STATION_POS);
        GrabberSubsystem.getInstance().set(-0.7);
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(3_000);

        GrabberSubsystem.getInstance().set(-0.1);
        GrabberSubsystem.getInstance().orientPos(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

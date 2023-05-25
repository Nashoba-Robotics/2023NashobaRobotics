package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tabs;
import frc.robot.Robot.RobotState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

//Cones ONLY!!
public class SingleStationIntakeCommand extends CommandBase{
    public SingleStationIntakeCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        // ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        // ArmSubsystem.getInstance().setDefaultAcceleration();

        ArmSubsystem.getInstance().setPivotCruiseVelocity(100);
        ArmSubsystem.getInstance().setPivotAcceleration(293);

        ArmSubsystem.getInstance().pivot(Constants.Arm.SINGLE_STATION_ANGLE);
        ArmSubsystem.getInstance().extendNU(Constants.Arm.SINGLE_STATION_EXTEND_NU);

        GrabberSubsystem.getInstance().orientPos(Constants.Grabber.SINGLE_STATION_POS);
        GrabberSubsystem.getInstance().set(Constants.Grabber.CONE_INTAKE_SPEED);
        GrabberSubsystem.getInstance().setCurrentLimit(true);

        Tabs.Comp.setExtendTarget(Constants.Arm.SINGLE_STATION_EXTEND_NU);
        Tabs.Comp.setPivotTarget(Constants.Arm.SINGLE_STATION_ANGLE);
        Tabs.Comp.setWristTarget(Constants.Grabber.SINGLE_STATION_POS);
    }

    @Override
    public void execute() {
        // if(Math.abs(GrabberSubsystem.getInstance().getOrientPos() - (-6)) < 0.5){
        //     GrabberSubsystem.getInstance().setOrientBrakeMode(IdleMode.kCoast);
        //     GrabberSubsystem.getInstance().setOrientSpeed(0);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        if(Robot.state == RobotState.OK && ArmSubsystem.getInstance().pivotStopped()) ArmSubsystem.getInstance().resetPivotNU();
        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(Constants.Arm.EXTEND_REST_NU);

        GrabberSubsystem.getInstance().setCurrentLimit(25, 25, 0.1);

        // GrabberSubsystem.getInstance().set(Constants.Grabber.CONE_HOLD_SPEED);
        // GrabberSubsystem.getInstance().orientPos(0);

        // GrabberSubsystem.getInstance().setOrientBrakeMode(IdleMode.kBrake);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands.intake;

import com.revrobotics.CANSparkMax.IdleMode;

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
        // ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        // ArmSubsystem.getInstance().setDefaultAcceleration();

        ArmSubsystem.getInstance().setPivotCruiseVelocity(400_000);
        ArmSubsystem.getInstance().setPivotAcceleration(60_000);

        ArmSubsystem.getInstance().pivot(76 * Constants.TAU/360);
        ArmSubsystem.getInstance().extendNU(0);

        GrabberSubsystem.getInstance().orientPos(-6);
        GrabberSubsystem.getInstance().set(Constants.Grabber.CONE_INTAKE_SPEED);
        GrabberSubsystem.getInstance().setCurrentLimit(true);

        // Tabs.Comp.setExtendTarget(Constants.Arm.DOUBLE_STATION_EXTEND_NU);
        // Tabs.Comp.setPivotTarget(Constants.Arm.DOUBLE_STATION_ANGLE);
        // Tabs.Comp.setWristTarget(Constants.Grabber.DOUBLE_STATION_POS);
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
        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(3_000);

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

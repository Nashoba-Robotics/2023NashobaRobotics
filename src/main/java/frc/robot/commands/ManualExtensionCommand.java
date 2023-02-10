package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ManualExtensionCommand extends CommandBase {
    Joystick extendJoystick;

    public ManualExtensionCommand() {
        extendJoystick = new Joystick(2);
        addRequirements(ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().set(0);
    }

    @Override
    public void execute() {
        ArmSubsystem.getInstance().set(-extendJoystick.getY());
        SmartDashboard.putNumber("Stator Current", ArmSubsystem.getInstance().getStatorCurrent());
        SmartDashboard.putNumber("Supply Current", ArmSubsystem.getInstance().getSupplyCurrent());
        SmartDashboard.putNumber("Extend NU", ArmSubsystem.getInstance().getPos());
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().set(0);
        ArmSubsystem.getInstance().extend(ArmSubsystem.getInstance().getPos());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

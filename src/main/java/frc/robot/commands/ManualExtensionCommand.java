package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ManualExtensionCommand extends CommandBase {
    Joystick extendJoystick;

    public ManualExtensionCommand() {
        extendJoystick = new Joystick(5);
        addRequirements(ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().set(0);
    }

    @Override
    public void execute() {
        double y = extendJoystick.getX();
        y = Math.abs(y) < 0.1 ? 0 : (y-0.1)/0.9;
        ArmSubsystem.getInstance().set(Math.abs(y)>0.3 ? 0 : -y);
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

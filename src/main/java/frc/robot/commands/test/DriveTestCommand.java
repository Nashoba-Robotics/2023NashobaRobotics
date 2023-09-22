package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.util.JoystickValues;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveTestCommand extends CommandBase{
    public DriveTestCommand(){
        addRequirements(SwerveDriveSubsystem.getInstance());
    }
    @Override
    public void execute() {
        SwerveDriveSubsystem.getInstance().set(new JoystickValues(0, 0.1), 0);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    }
}

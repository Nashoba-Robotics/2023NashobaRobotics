package frc.robot.commands.auto.balance.routine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class waitUntilLevel extends CommandBase {
    
    public waitUntilLevel() {
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void execute() {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return SwerveDriveSubsystem.getInstance().isLevel();
    }

}

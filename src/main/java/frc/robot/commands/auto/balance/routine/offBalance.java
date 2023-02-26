package frc.robot.commands.auto.balance.routine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class offBalance extends CommandBase {
    
    public offBalance() {
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SwerveDriveSubsystem.getInstance().set(0.3, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return SwerveDriveSubsystem.getInstance().isLevel();
    }

}

package frc.robot.commands.auto.balance.routine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class onToBalance extends CommandBase {
    
    public onToBalance() {
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SwerveDriveSubsystem.getInstance().set(0.7, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return SwerveDriveSubsystem.getInstance().notLevel();
    }

}

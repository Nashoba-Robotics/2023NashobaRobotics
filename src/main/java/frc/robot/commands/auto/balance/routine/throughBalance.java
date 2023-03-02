package frc.robot.commands.auto.balance.routine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class throughBalance extends CommandBase {
    
    public throughBalance() {
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SwerveDriveSubsystem.getInstance().set(-0.3, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return SwerveDriveSubsystem.getInstance().levelNegative();
    }

}

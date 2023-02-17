package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BalanceCommand extends CommandBase{
    public BalanceCommand(){
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SwerveDriveSubsystem.getInstance().setDesiredLevel(1, 1);
    }

    @Override
    public void execute() {
        SwerveDriveSubsystem.getInstance().set(
            1.5*SwerveDriveSubsystem.getInstance().getChange(), 
            0,
            0
        );
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return SwerveDriveSubsystem.getInstance().balanced();
    }
}

package frc.robot.commands.auto.balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BalanceCommand extends CommandBase{
    public enum Balance{
        QUICK,
        SLOW
    }

    private Balance b = Balance.SLOW;
    public BalanceCommand(){
        addRequirements(SwerveDriveSubsystem.getInstance());
    }
    public BalanceCommand(Balance b){
        this.b = b;
    }

    @Override
    public void initialize() {
        SwerveDriveSubsystem.getInstance().setDesiredLevel(1, 1);
        switch(b){
            case QUICK:
                SwerveDriveSubsystem.getInstance().setBalancePID(
                    Constants.Swerve.Balance.FAST_K_P, 
                    Constants.Swerve.Balance.FAST_K_I, 
                    Constants.Swerve.Balance.FAST_K_D
                );
                break;
            case SLOW:
                SwerveDriveSubsystem.getInstance().setBalancePID(
                    Constants.Swerve.Balance.SLOW_K_P, 
                    Constants.Swerve.Balance.SLOW_K_I, 
                    Constants.Swerve.Balance.SLOW_K_D
                );
                break;
        }
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
        
    }

    @Override
    public boolean isFinished() {
        return SwerveDriveSubsystem.getInstance().balanced();
    }
}

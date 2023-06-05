package frc.robot.commands.auto.balance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NewTabs;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BetterBalanceCommand extends CommandBase{
    PIDController balanceController;

    public BetterBalanceCommand(){
        balanceController = new PIDController(0, 0, 0);
        balanceController.setSetpoint(0);
        balanceController.setTolerance(0);

    }

    @Override
    public void execute() {
        double kP = NewTabs.getDouble("Balance Test", "kP", 0);
        double kI = NewTabs.getDouble("Balance Test", "kI", 0);
        double kD = NewTabs.getDouble("Balance Test", "kD", 0);

        double tolerance = NewTabs.getDouble("Balance Test", "Tolerance", 0);

        balanceController.setP(kP); //Fast_KP = 0.01;
        balanceController.setI(kI);
        balanceController.setD(kD);

        balanceController.setTolerance(tolerance);

        //Check whether positive or negative going up charge station from outside
        double change = balanceController.calculate(SwerveDriveSubsystem.getInstance().getRoll());
        SwerveDriveSubsystem.getInstance().set(
            change, 
            0,
            0
        );
    }
}

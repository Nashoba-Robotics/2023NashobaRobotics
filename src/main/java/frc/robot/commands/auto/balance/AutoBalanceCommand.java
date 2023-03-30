package frc.robot.commands.auto.balance;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.balance.BalanceCommand.Balance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoBalanceCommand extends SequentialCommandGroup{
    public AutoBalanceCommand(){
        addCommands(
            new InstantCommand(
                () -> ArmSubsystem.getInstance().pivot(-Math.PI/2),
                ArmSubsystem.getInstance()
            ),
            new BalanceCommand(Balance.QUICK),
            new WaitCommand(0.25),
            new InstantCommand(() -> ArmSubsystem.getInstance().pivot(-Constants.TAU/4), ArmSubsystem.getInstance()),
            new BalanceCommand(Balance.SLOW),
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().brake(), SwerveDriveSubsystem.getInstance())
            // new InstantCommand(
            //     () -> SwerveDriveSubsystem.getInstance().brake(),
            //     SwerveDriveSubsystem.getInstance()
            // )
        );
    }
}

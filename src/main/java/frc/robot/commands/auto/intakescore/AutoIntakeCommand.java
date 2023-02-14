package frc.robot.commands.auto.intakescore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoIntakeCommand extends SequentialCommandGroup {
    
    public AutoIntakeCommand() {
        addCommands(
            new InstantCommand(() -> {
                ArmSubsystem.getInstance().pivot(100 * Constants.TAU/360);
                ArmSubsystem.getInstance().extendNU(2_000);
            }),
            new WaitCommand(5),
            new InstantCommand(() -> {
                ArmSubsystem.getInstance().pivot(90 * Constants.TAU/360);
                ArmSubsystem.getInstance().extendNU(0);
            }),
            new WaitCommand(5),
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().set(-0.7);
                GrabberSubsystem.getInstance().orientPos(-3);
                ArmSubsystem.getInstance().pivot(113 * Constants.TAU/360);
                ArmSubsystem.getInstance().extendNU(1_000);
            }),
            new WaitCommand(5),
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().set(-0.1);
                GrabberSubsystem.getInstance().orientPos(0);
                ArmSubsystem.getInstance().pivot(0);
                ArmSubsystem.getInstance().extendNU(0);
            })
        );
    }

}
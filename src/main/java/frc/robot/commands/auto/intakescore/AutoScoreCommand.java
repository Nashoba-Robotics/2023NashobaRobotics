package frc.robot.commands.auto.intakescore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.score.AutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoScoreCommand extends SequentialCommandGroup {
    
    public AutoScoreCommand() {
        addCommands(
            new AutoDirectionalPrepHeightCommand(TargetLevel.HIGH).withTimeout(1.5),
            new ScoreCommand().withTimeout(1),
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().stop();
                ArmSubsystem.getInstance().pivot(0);
                ArmSubsystem.getInstance().extendNU(0);
            }, ArmSubsystem.getInstance(), GrabberSubsystem.getInstance())
        );
    }

}

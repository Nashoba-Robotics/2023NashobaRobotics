package frc.robot.commands.auto.intakescore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.score.AutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.ScoreConeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoScoreCommand extends SequentialCommandGroup {
    
    public AutoScoreCommand() {
        addCommands(
            new AutoDirectionalPrepHeightCommand(TargetLevel.HIGH, false).withTimeout(0.8),
            new ScoreConeCommand().withTimeout(0.7),
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().stop();
                ArmSubsystem.getInstance().pivot(0);
                ArmSubsystem.getInstance().extendNU(0);
            }, ArmSubsystem.getInstance(), GrabberSubsystem.getInstance())
        );
    }

}

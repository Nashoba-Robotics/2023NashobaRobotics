package frc.robot.commands.auto.intakescore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.score.CubeAutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.ScoreCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoScoreCubeCommand extends SequentialCommandGroup {
    
    public AutoScoreCubeCommand() {
        addCommands(
            new CubeAutoDirectionalPrepHeightCommand(TargetLevel.HIGH, false).withTimeout(1.4),
            new ScoreCubeCommand().withTimeout(0.4)
        );
    }

}

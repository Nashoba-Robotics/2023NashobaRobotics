package frc.robot.commands.auto.intakescore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.score.CubeAutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.ScoreCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoScoreCubeHeightCommand extends SequentialCommandGroup {
    
    public AutoScoreCubeHeightCommand(TargetLevel height) {
        addCommands(
            new CubeAutoDirectionalPrepHeightCommand(height, true).withTimeout(1.4),
            new ScoreCubeCommand().withTimeout(0.4)
        );
    }

}

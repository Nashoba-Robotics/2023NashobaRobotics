package frc.robot.commands.auto.intakescore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.score.CubeAutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.ScoreCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoScoreCubeCommand extends SequentialCommandGroup {
    
    public AutoScoreCubeCommand() {
        addCommands(
            new CubeAutoDirectionalPrepHeightCommand(TargetLevel.HIGH).withTimeout(2),
            new ScoreCubeCommand().withTimeout(1),
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().stop();
                ArmSubsystem.getInstance().pivot(0);
                ArmSubsystem.getInstance().extendNU(0);
            }, ArmSubsystem.getInstance(), GrabberSubsystem.getInstance())
        );
    }

}

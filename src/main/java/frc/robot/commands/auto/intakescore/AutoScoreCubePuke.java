package frc.robot.commands.auto.intakescore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.score.CubeAutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.PukeCommand;
import frc.robot.commands.score.ScoreCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoScoreCubePuke extends SequentialCommandGroup{
    public AutoScoreCubePuke(){
        addCommands(
            new CubeAutoDirectionalPrepHeightCommand(TargetLevel.HIGH).withTimeout(2),
            new ScoreCubeCommand().withTimeout(1),
            // new PukeCommand().withTimeout(2),
            new InstantCommand(
                () -> {
                    GrabberSubsystem.getInstance().orientPos(30);
                    GrabberSubsystem.getInstance().set(0);
                }, GrabberSubsystem.getInstance()
            ),
            new WaitCommand(2),
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().stop();
                ArmSubsystem.getInstance().pivot(0);
                ArmSubsystem.getInstance().extendNU(0);
            }, ArmSubsystem.getInstance(), GrabberSubsystem.getInstance())
        );
    }
}

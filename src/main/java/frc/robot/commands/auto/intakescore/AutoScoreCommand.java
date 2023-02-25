package frc.robot.commands.auto.intakescore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Grabber;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.score.PrepHeightCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoScoreCommand extends SequentialCommandGroup {
    
    public AutoScoreCommand() {
        addCommands(
            //new PrepHeightCommand(TargetLevel.HIGH).withTimeout(0.5),
            new SequentialCommandGroup(
                new InstantCommand(()-> ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU)),
                new WaitCommand(0.2),
                new InstantCommand(() -> ArmSubsystem.getInstance().pivot(Constants.Arm.HIGH_ANGLE))
            ),
            new WaitCommand(1.2),
            new ScoreCommand().withTimeout(1),
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().stop();
                ArmSubsystem.getInstance().pivot(0);
                ArmSubsystem.getInstance().extendNU(0);
            }, ArmSubsystem.getInstance(), GrabberSubsystem.getInstance())
        );
    }

}

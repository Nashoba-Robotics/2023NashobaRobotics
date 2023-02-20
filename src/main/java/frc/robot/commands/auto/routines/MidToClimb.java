package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoPaths;
import frc.robot.Constants;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MidToClimb extends SequentialCommandGroup{
    public MidToClimb(){
        addCommands(
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/4), SwerveDriveSubsystem.getInstance()),
            new InstantCommand(() -> GrabberSubsystem.getInstance().zeroWrist(), GrabberSubsystem.getInstance()),
            new AutoScoreCommand(),
            new FollowPathCommand(AutoPaths.midAToClimb).until(SwerveDriveSubsystem.getInstance()::notLevel),
            new BalanceCommand()
        );
    }
}
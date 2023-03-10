package frc.robot.commands.auto.routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeCubeCommand;
import frc.robot.commands.auto.balance.AutoBalanceCommand;
import frc.robot.commands.auto.balance.routine.backToBalance;
import frc.robot.commands.auto.balance.routine.offBalance;
import frc.robot.commands.auto.balance.routine.onToBalance;
import frc.robot.commands.auto.balance.routine.throughBalance;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.move.TranslateToCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

public class MidToClimb extends SequentialCommandGroup{
    PathPlannerTrajectory path = PathPlanner.loadPath("BLUE-midA-climb", new PathConstraints(4, 3));

    public MidToClimb(){

        Command translateTo = new TranslateToCommand(new Translation2d(1.55, 0.25), Rotation2d.fromRadians(Constants.TAU/2));
        Command translateBack = new TranslateToCommand(new Translation2d(-4, -0.35), Rotation2d.fromRadians(Constants.TAU/2));

        addCommands(
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().zeroWrist();
                SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
                SwerveDriveSubsystem.getInstance().resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180)));
                ArmSubsystem.getInstance().pivot(0);
            // }, GrabberSubsystem.getInstance(), SwerveDriveSubsystem.getInstance()),
            }, GrabberSubsystem.getInstance()),
            new AutoScoreCommand(), //<-- This makes us tip a bit
            new WaitCommand(1), //<-- This makes sure the tip does not mess up the end conditions :)
            new ParallelCommandGroup(
                new onToBalance(),
                new InstantCommand(
                    () -> ArmSubsystem.getInstance().pivot(-Constants.TAU/4),
                    ArmSubsystem.getInstance()
                )
            ),
            new throughBalance(),
            new offBalance(),
            new WaitCommand(0.1),
            new ParallelCommandGroup(
                translateTo.withTimeout(2.5),
                new IntakeCubeCommand(true).withTimeout(1.5)
            ),
            translateBack.until(SwerveDriveSubsystem.getInstance()::notLevel),
            // new InstantCommand(() -> CandleSubsystem.getInstance().set(CandleState.WANT_CUBE))
            // new backToBalance(),
            new AutoBalanceCommand()
        );
    }
}
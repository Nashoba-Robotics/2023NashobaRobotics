package frc.robot.commands.auto.routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoPaths;
import frc.robot.Constants;
import frc.robot.commands.auto.balance.BalanceCommand;
import frc.robot.commands.auto.balance.BalanceCommand.Balance;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MidToClimb extends SequentialCommandGroup{
    PathPlannerTrajectory path = PathPlanner.loadPath("BLUE-midA-climb", new PathConstraints(4, 3));
    public MidToClimb(){
        addCommands(
            new InstantCommand(() -> GrabberSubsystem.getInstance().zeroWrist(), GrabberSubsystem.getInstance()),
            new AutoScoreCommand(),
            new FollowPathCommand(path),
            new BalanceCommand(Balance.QUICK),
            new BalanceCommand(Balance.SLOW),
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().brake(), SwerveDriveSubsystem.getInstance())
        );
    }
}
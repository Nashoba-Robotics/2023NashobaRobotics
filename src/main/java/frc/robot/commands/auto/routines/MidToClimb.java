package frc.robot.commands.auto.routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.balance.AutoBalanceCommand;
import frc.robot.commands.auto.balance.routine.backToBalance;
import frc.robot.commands.auto.balance.routine.offBalance;
import frc.robot.commands.auto.balance.routine.onToBalance;
import frc.robot.commands.auto.balance.routine.throughBalance;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MidToClimb extends SequentialCommandGroup{
    PathPlannerTrajectory path = PathPlanner.loadPath("BLUE-midA-climb", new PathConstraints(4, 3));

    public MidToClimb(){
        addCommands(
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().zeroWrist();
                SwerveDriveSubsystem.getInstance().setGyro(0);
            }, GrabberSubsystem.getInstance(), SwerveDriveSubsystem.getInstance()),
            new AutoScoreCommand(),
            new onToBalance(),
            new throughBalance(),
            new offBalance(),
            new WaitCommand(0.25),
            new backToBalance(),
            new AutoBalanceCommand()
        );
    }
}
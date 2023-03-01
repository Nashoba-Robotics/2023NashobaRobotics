package frc.robot.commands.auto.routines;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCubeCommand;
import frc.robot.commands.auto.balance.AutoBalanceCommand;
import frc.robot.commands.auto.balance.routine.backToBalance;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class RightTo3ToBalance extends SequentialCommandGroup {
    
    public RightTo3ToBalance() {
        Map<String, Command> map = new HashMap<>();
        map.put("Start Intake", new IntakeCubeCommand());
        map.put("Stop Intake", new InstantCommand(
            () -> {
                ArmSubsystem.getInstance().pivot(0);
                GrabberSubsystem.getInstance().set(-0.1);
            },
            ArmSubsystem.getInstance(),
            GrabberSubsystem.getInstance()
        ));
        PathPlannerTrajectory path = PathPlanner.loadPath("rightA-0-climb", new PathConstraints(2, 2));

        FollowPathWithEvents command = new FollowPathWithEvents(
            new FollowPathCommand(path),
            path.getMarkers(),
            map);
            
        addCommands(
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().zeroWrist();
                ArmSubsystem.getInstance().resetPivotNU();
            }, GrabberSubsystem.getInstance(), ArmSubsystem.getInstance()),
            new AutoScoreCommand(),
            command,
            new backToBalance(),
            new AutoBalanceCommand()
        );
    }

}

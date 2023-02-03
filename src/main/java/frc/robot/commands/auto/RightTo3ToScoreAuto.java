package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RightTo3ToScoreAuto extends SequentialCommandGroup{
    List<PathPlannerTrajectory> paths;
    public RightTo3ToScoreAuto(){
        addRequirements(SwerveDriveSubsystem.getInstance());
        paths = PathPlanner.loadPathGroup("rightTo3ToScore", new PathConstraints(4, 2));
        addCommands(
            new FollowPathCommand(paths.get(0))
        );
    }
}

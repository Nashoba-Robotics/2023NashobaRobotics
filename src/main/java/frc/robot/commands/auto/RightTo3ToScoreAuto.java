package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RightTo3ToScoreAuto extends SequentialCommandGroup{
    public RightTo3ToScoreAuto(){
        addRequirements(SwerveDriveSubsystem.getInstance());
        
        // PathPlannerTrajectory.transformTrajectoryForAlliance(null, Alliance.Red);
        addCommands(

        );
    }
}

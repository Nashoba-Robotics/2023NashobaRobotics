package frc.robot.commands.test;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveToTestCommand extends CommandBase {
    
    @Override
    public void initialize() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(2, 1.5), 
            List.of(
                new PathPoint(SwerveDriveSubsystem.getInstance().getPose().getTranslation(), Rotation2d.fromDegrees(180), SwerveDriveSubsystem.getInstance().getPose().getRotation()),
                new PathPoint(new Translation2d(1.684, 2.884), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0))
            )
        );

        CommandScheduler.getInstance().schedule(new FollowPathCommand(trajectory));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

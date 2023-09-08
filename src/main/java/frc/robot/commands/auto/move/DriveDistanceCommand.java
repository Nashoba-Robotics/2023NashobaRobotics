package frc.robot.commands.auto.move;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveDistanceCommand extends SequentialCommandGroup {
    
    public DriveDistanceCommand(double distance) {
        PathConstraints constraints = new PathConstraints(1.5, 1.5);

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            constraints, 
            List.of(
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(distance, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
            )
        );
        addCommands(
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            }, SwerveDriveSubsystem.getInstance()),
            new FollowPathCommand(trajectory),
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().set(0, 0, 0);
            })


        );
    }

}

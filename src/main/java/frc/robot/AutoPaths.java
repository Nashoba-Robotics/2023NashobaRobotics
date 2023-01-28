package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public final class AutoPaths {
    
    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Constants.Swerve.Auto.MAX_SPEED,
            Constants.Swerve.Auto.MAX_ACCELERATION
        ).setKinematics(Constants.Swerve.KINEMATICS);

    public static final PathPlannerTrajectory testTrajectory = PathPlanner.generatePath(
        new PathConstraints(Constants.Swerve.Auto.MAX_SPEED, Constants.Swerve.Auto.MAX_ACCELERATION), 
        List.of(
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(90)),
            new PathPoint(new Translation2d(2, 0), Rotation2d.fromDegrees(90))
        )
    );

    public static final Trajectory leftTo0 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.837, 4.990, Rotation2d.fromDegrees(0)),
        List.of(),
        new Pose2d(6.6, 4.99, Rotation2d.fromDegrees(0)),
        trajectoryConfig
        );

    public static final Trajectory rightTo3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.777, 0.463, Rotation2d.fromDegrees(-90)), 
        null, 
        new Pose2d(6.759, 0.904, Rotation2d.fromDegrees(-90)), 
        trajectoryConfig
    );

    

}

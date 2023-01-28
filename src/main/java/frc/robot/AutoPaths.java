package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public final class AutoPaths {
    
    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Constants.Swerve.Auto.MAX_SPEED,
            Constants.Swerve.Auto.MAX_ACCELERATION
        ).setKinematics(Constants.Swerve.KINEMATICS);

    public static final Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(),
        new Pose2d(3, 2, Rotation2d.fromDegrees(0)),
        trajectoryConfig
        );

}

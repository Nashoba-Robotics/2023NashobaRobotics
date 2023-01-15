package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public final class Paths {
    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Constants.Swerve.MAX_SPEED,
            Constants.Swerve.MAX_ACCELERATION
        ).setKinematics(Constants.Swerve.KINEMATICS);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(),
        new Pose2d(0, 1, Rotation2d.fromDegrees(0)),
        trajectoryConfig
        );

}

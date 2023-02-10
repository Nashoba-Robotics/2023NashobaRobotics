package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/*
    Stores trajectories for auto
*/
public final class AutoPaths {

    public static final PathPlannerTrajectory testTrajectory = PathPlanner.generatePath(
        new PathConstraints(Constants.Swerve.Auto.MAX_SPEED, Constants.Swerve.Auto.MAX_ACCELERATION), 
        List.of(
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90)),
            new PathPoint(new Translation2d(2, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90))
        )
    );

}

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
//NOT IN USE
public final class AutoPaths {
    public static final PathPlannerTrajectory testTrajectory = PathPlanner.generatePath(
        new PathConstraints(Constants.Swerve.Auto.MAX_SPEED, Constants.Swerve.Auto.MAX_ACCELERATION), 
        List.of(
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(-90)),
            new PathPoint(new Translation2d(0, 2), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(-90))
        )
    );

    public static final PathPlannerTrajectory leftAToPiece0 = PathPlanner.generatePath(
        new PathConstraints(Constants.Swerve.Auto.MAX_SPEED, Constants.Swerve.Auto.MAX_ACCELERATION),
        List.of(
            new PathPoint(FieldLocations.Blue.LEFT_A, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90)),
            new PathPoint(FieldLocations.Blue.PIECE_0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-90))
        )
    );

    public static final PathPlannerTrajectory piece0ToLeftC = PathPlanner.generatePath(
        new PathConstraints(Constants.Swerve.Auto.MAX_SPEED, Constants.Swerve.Auto.MAX_ACCELERATION),
        true,
        List.of(
            new PathPoint(FieldLocations.Blue.PIECE_0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-90)),
            new PathPoint(FieldLocations.Blue.LEFT_C, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90))
        )
    );

    public static final PathPlannerTrajectory piece0ToClimb = PathPlanner.generatePath(
        new PathConstraints(Constants.Swerve.Auto.MAX_SPEED, Constants.Swerve.Auto.MAX_ACCELERATION),
        true,
        List.of(
            new PathPoint(FieldLocations.Blue.PIECE_0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-90)),
            new PathPoint(FieldLocations.Blue.OUTER_BALANCE, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90))
        )
    );

    public static final PathPlannerTrajectory midAToClimb = PathPlanner.generatePath(
        new PathConstraints(Constants.Swerve.Auto.MAX_SPEED, Constants.Swerve.Auto.MAX_ACCELERATION),
        List.of(
            new PathPoint(FieldLocations.Blue.MID_A, Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(90)),
            new PathPoint(FieldLocations.Blue.INNER_BALANCE, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90))
        )
    );

    public static final PathPlannerTrajectory midBToClimb = PathPlanner.generatePath(
        new PathConstraints(Constants.Swerve.Auto.MAX_SPEED, Constants.Swerve.Auto.MAX_ACCELERATION),
        List.of(
            new PathPoint(FieldLocations.Blue.MID_B, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90)),
            new PathPoint(FieldLocations.Blue.INNER_BALANCE, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90))
        )
    );

    public static final PathPlannerTrajectory rightCToPiece3 = PathPlanner.generatePath(
        new PathConstraints(Constants.Swerve.Auto.MAX_SPEED, Constants.Swerve.Auto.MAX_ACCELERATION),
        List.of(
            new PathPoint(FieldLocations.Blue.RIGHT_C, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90)),
            new PathPoint(FieldLocations.Blue.PIECE_3, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-90))
        )
    );

    public static final PathPlannerTrajectory piece3ToRightA = PathPlanner.generatePath(
        new PathConstraints(Constants.Swerve.Auto.MAX_SPEED, Constants.Swerve.Auto.MAX_ACCELERATION),
        true,
        List.of(
            new PathPoint(FieldLocations.Blue.PIECE_3, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-90)),
            new PathPoint(FieldLocations.Blue.RIGHT_A, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90))
        )
    );

    public static final PathPlannerTrajectory piece3ToClimb = PathPlanner.generatePath(
        new PathConstraints(Constants.Swerve.Auto.MAX_SPEED, Constants.Swerve.Auto.MAX_ACCELERATION),
        true,
        List.of(
            new PathPoint(FieldLocations.Blue.PIECE_3, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-90)),
            new PathPoint(FieldLocations.Blue.OUTER_BALANCE, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90))
        )
    );

}

package frc.robot.commands.auto.move;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ToRightScore extends CommandBase {

    private final double[] leftPos = {0.8, 4.4};
    private final double[] midPos = {0.8, 2.7};
    private final double[] rightPos = {0.8, 1};
    
    @Override
    public void initialize() {
        double robotX = SwerveDriveSubsystem.getInstance().getPose().getX();
        double robotY = SwerveDriveSubsystem.getInstance().getPose().getY();
        double leftDistance = Math.sqrt((robotX-leftPos[0])*(robotX-leftPos[0]) + (robotY-leftPos[1])*(robotY-leftPos[1]));
        double midDistance = Math.sqrt((robotX-midPos[0])*(robotX-midPos[0]) + (robotY-midPos[1])*(robotY-midPos[1]));
        double rightDistance = Math.sqrt((robotX-rightPos[0])*(robotX-rightPos[0]) + (robotY-rightPos[1])*(robotY-rightPos[1]));
        double min = Math.min(Math.min(leftDistance, midDistance), rightDistance);

        PathPoint endpoint;

        if(min == leftDistance) {
            endpoint = new PathPoint(new Translation2d(1.75, 3.85), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180));
        } else if(min == midDistance) {
            endpoint = new PathPoint(new Translation2d(1.75, 2.15), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180));
        } else {
            endpoint = new PathPoint(new Translation2d(1.75, 0.45), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180));
        }

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(1, 1), 
            List.of(
                new PathPoint(SwerveDriveSubsystem.getInstance().getPose().getTranslation(), Rotation2d.fromDegrees(180), SwerveDriveSubsystem.getInstance().getPose().getRotation()),
                endpoint
            )
        );

        CommandScheduler.getInstance().schedule(new FollowPathCommand(trajectory));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

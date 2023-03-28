package frc.robot.commands.auto.move;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TranslateCameraCommand extends CommandBase {

    private Command command;

    private PathPlannerTrajectory trajectory;

    @Override
    public void initialize() {
        double tx = LimelightSubsystem.getInstance().getTX();
        double ty = LimelightSubsystem.getInstance().getTY();
        Pose2d currPosition = SwerveDriveSubsystem.getInstance().getPose();

        double movement = txTyToMeters(tx, ty);

        trajectory = PathPlanner.generatePath(
            new PathConstraints(3.5, 2.5), 
            List.of(
                new PathPoint(currPosition.getTranslation(), Rotation2d.fromRadians(Constants.TAU/4), currPosition.getRotation()),
                new PathPoint(new Translation2d(currPosition.getX(), currPosition.getY() + movement), Rotation2d.fromRadians(Constants.TAU/4), currPosition.getRotation())
            )
        );

        command = new FollowPathCommand(trajectory, true);

        CommandScheduler.getInstance().schedule(command);
    }

    @Override
    public void end(boolean interrupted) {
        command.cancel();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    public double txTyToMeters(double tx, double ty) {
        return tx * 1;
    }

}

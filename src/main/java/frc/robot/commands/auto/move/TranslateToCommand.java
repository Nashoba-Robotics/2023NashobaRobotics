package frc.robot.commands.auto.move;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TranslateToCommand extends CommandBase {

    private Translation2d blueTranslation;
    private Translation2d redTranslation;
    private Translation2d translation;
    private Rotation2d endRotation;
    private Command command;

    private PathPlannerTrajectory trajectory;
    private PathConstraints constraints;

    private Rotation2d heading;

    public TranslateToCommand(Translation2d translation, Rotation2d endRotation, PathConstraints constraints) {
        // addRequirements(SwerveDriveSubsystem.getInstance());
        blueTranslation = translation;
        redTranslation = new Translation2d(translation.getX(), -translation.getY());
        this.translation = blueTranslation;
        this.endRotation = endRotation;
        this.constraints = constraints;
        this.heading = null;
    }

    public TranslateToCommand(Translation2d translation, Rotation2d endRotation, Rotation2d heading, PathConstraints constraints) {
        // addRequirements(SwerveDriveSubsystem.getInstance());
        blueTranslation = translation;
        redTranslation = new Translation2d(translation.getX(), -translation.getY());
        this.translation = blueTranslation;
        this.endRotation = endRotation;
        this.constraints = constraints;
        this.heading = heading;
    }

    @Override
    public void initialize() {
        Pose2d currPosition = SwerveDriveSubsystem.getInstance().getPose();

        if(heading == null) {
            heading = translation.getX() < 0 ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
        }

        if(DriverStation.getAlliance() == Alliance.Red) {
            translation = redTranslation;
        } else {
            translation = blueTranslation;
        }

        trajectory = PathPlanner.generatePath(
            constraints, 
            List.of(
                new PathPoint(currPosition.getTranslation(), heading, currPosition.getRotation()),
                new PathPoint(new Translation2d(currPosition.getX() + translation.getX(), currPosition.getY() + translation.getY()), heading, endRotation)
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

}

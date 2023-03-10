package frc.robot.commands.auto.move;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TranslateToCommand extends CommandBase {

    private Translation2d translation;
    private Rotation2d endRotation;
    private Command command;

    private PathPlannerTrajectory trajectory;

    private Timer timer;


    public TranslateToCommand(Translation2d translation, Rotation2d endRotation) {
        // addRequirements(SwerveDriveSubsystem.getInstance());
        this.translation = translation;
        this.endRotation = endRotation;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        Pose2d currPosition = SwerveDriveSubsystem.getInstance().getPose();

        Rotation2d heading = translation.getX() < 0 ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

        trajectory = PathPlanner.generatePath(
            new PathConstraints(3.5, 2), 
            List.of(
                new PathPoint(currPosition.getTranslation(), heading, currPosition.getRotation()),
                new PathPoint(new Translation2d(currPosition.getX() + translation.getX(), currPosition.getY() + translation.getY()), heading, endRotation)
            )
        );

        command = new FollowPathCommand(trajectory, true);
        timer.reset();
        timer.start();

        CommandScheduler.getInstance().schedule(command);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Time", timer.get());
        SmartDashboard.putBoolean("Ben", isFinished());
    }
    @Override
    public void end(boolean interrupted) {
        command.cancel();
        SmartDashboard.putBoolean("AAAAAAAAAAAAAAAAAAAAAHHHHHHHHHH", true);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

}

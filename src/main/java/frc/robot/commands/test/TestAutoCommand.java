package frc.robot.commands.test;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoPaths;
import frc.robot.Constants;
import frc.robot.Constants.Grabber;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TestAutoCommand extends SequentialCommandGroup {
    
    public TestAutoCommand() {

        PathPlannerTrajectory path = PathPlanner.loadPath("BLUE-rightC-3-rightA", new PathConstraints(2, 2));

        addCommands(
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/4), SwerveDriveSubsystem.getInstance()),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().resetOdometry(
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                        path,
                        DriverStation.getAlliance()).getInitialHolonomicPose()
                    );
            }),
            new WaitCommand(0.5),
            new FollowPathCommand(path)
        );
    }
}

package frc.robot.commands.test;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoPaths;
import frc.robot.Constants;
import frc.robot.Constants.Grabber;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.commands.auto.move.TranslateToCommand;
import frc.robot.commands.intake.IntakeConeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TestAutoCommand extends SequentialCommandGroup {
    
    public TestAutoCommand() {
        Map<String, Command> map = new HashMap<>();
        // map.put("Start Intake", new IntakeCommand(true));
        // map.put("Stop Intake", new InstantCommand(
        //     () -> {
        //         ArmSubsystem.getInstance().pivot(0);
        //         GrabberSubsystem.getInstance().set(-0.1);
        //     },
        //     ArmSubsystem.getInstance(),
        //     GrabberSubsystem.getInstance()
        // ));

        PathPlannerTrajectory path = PathPlanner.loadPath("testPath", new PathConstraints(2, 2));
        // FollowPathWithEvents command = new FollowPathWithEvents(
        //     new FollowPathCommand(path),
        //     path.getMarkers(),
        //     map);
        addCommands(
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().setGyro(0), SwerveDriveSubsystem.getInstance()),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().resetOdometry(
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                        path,
                        DriverStation.getAlliance()).getInitialHolonomicPose()
                    );
            }),
            new WaitCommand(0.5),
            new TranslateToCommand(new Translation2d(0, 2), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90), new PathConstraints(2, 1))
        );
    }
}

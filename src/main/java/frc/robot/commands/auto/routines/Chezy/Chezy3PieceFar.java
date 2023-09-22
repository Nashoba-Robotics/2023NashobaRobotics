package frc.robot.commands.auto.routines.Chezy;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.auto.intakescore.AutoBacksideCubeScoreCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCubeHeightCommand;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.commands.intake.IntakeCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Chezy3PieceFar extends SequentialCommandGroup {
    
    public Chezy3PieceFar() {
        HashMap<String, Command> map = new HashMap<>();
        map.put("Start Intake", new IntakeCubeCommand());
        map.put("Stop Intake", new InstantCommand(
            () -> {
                ArmSubsystem.getInstance().pivot(0);
                GrabberSubsystem.getInstance().set(-0.1);
            },
            ArmSubsystem.getInstance(),
            GrabberSubsystem.getInstance()
        ));
        map.put("High Cube Prep", new InstantCommand(
            () -> {
                ArmSubsystem.getInstance().pivot(60 * Constants.TAU/360);
            }, ArmSubsystem.getInstance()
        ));
        map.put("Mid Cube Prep", new InstantCommand(
            () -> {
                ArmSubsystem.getInstance().pivot(70 * Constants.TAU/360);
            }, ArmSubsystem.getInstance()
        ));

        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Chezy-3-piece-far",
            new PathConstraints(5, 3),
            new PathConstraints(5, 3)
            );

            FollowPathWithEvents path1 = new FollowPathWithEvents(
            new FollowPathCommand(path.get(0)),
            path.get(0).getMarkers(),
            map);
            FollowPathWithEvents path2 = new FollowPathWithEvents(
                new FollowPathCommand(path.get(1)),
                path.get(1).getMarkers(),
                map
            );

        addCommands(
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().zeroWrist();
                GrabberSubsystem.getInstance().set(0);
                SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
            }, GrabberSubsystem.getInstance(), SwerveDriveSubsystem.getInstance()),
            new AutoScoreCommand(),
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().resetOdometry(
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                        path.get(0),
                        DriverStation.getAlliance()).getInitialHolonomicPose()
                    );
            }, SwerveDriveSubsystem.getInstance()),
            path1,
            new AutoBacksideCubeScoreCommand(true),
            path2,
            new AutoBacksideCubeScoreCommand(false)
        );

    }

}

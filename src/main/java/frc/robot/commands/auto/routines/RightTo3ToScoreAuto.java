package frc.robot.commands.auto.routines;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCubeCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCubePuke;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.commands.intake.IntakeConeCommand;
import frc.robot.commands.intake.IntakeCubeCommand;
import frc.robot.commands.score.PukeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RightTo3ToScoreAuto extends SequentialCommandGroup{
    public RightTo3ToScoreAuto(){
        Map<String, Command> map = new HashMap<>();
        // map.put("Start Intake", new IntakeCubeCommand());
        // map.put("Stop Intake", new InstantCommand(
        //     () -> {
        //         ArmSubsystem.getInstance().pivot(0);
        //         GrabberSubsystem.getInstance().set(-0.1);    //Don't remember if this is supposed to be positive or negative
        //     },
        //     ArmSubsystem.getInstance(),
        //     GrabberSubsystem.getInstance()
        // ));
        // map.put("Prep Score Angle", new InstantCommand(() -> ArmSubsystem.getInstance().pivot(-22 * Constants.TAU/360), ArmSubsystem.getInstance()));
        
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("BLUE-rightC-3-rightA",
            new PathConstraints(2.5, 1.5),
            // new PathConstraints(3.5, 2.5),
            // new PathConstraints(3.5, 1.5),
            new PathConstraints(2.5, 1.5));
        
        FollowPathWithEvents path1 = new FollowPathWithEvents(
            new FollowPathCommand(path.get(0)),
            path.get(0).getMarkers(),
            map);

        FollowPathWithEvents path2 = new FollowPathWithEvents(
            new FollowPathCommand(path.get(1)),
            path.get(1).getMarkers(),
            map);

        // FollowPathWithEvents path3 = new FollowPathWithEvents(
        // new FollowPathCommand(path.get(2)),
        // path.get(2).getMarkers(),
        // map);

        // FollowPathWithEvents path4 = new FollowPathWithEvents(
        //     new FollowPathCommand(path.get(3)),
        //     path.get(3).getMarkers(),
        //     map);


        addCommands(
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().zeroWrist();
                GrabberSubsystem.getInstance().set(0);
                SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
            }, GrabberSubsystem.getInstance(), SwerveDriveSubsystem.getInstance()),
            new WaitCommand(0.05),
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().resetOdometry(
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                        path.get(0),
                        DriverStation.getAlliance()).getInitialHolonomicPose()
                    );
            }, SwerveDriveSubsystem.getInstance()),
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().resetOdometry(
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                        path.get(0),
                        DriverStation.getAlliance()).getInitialHolonomicPose()
                    );
            }, SwerveDriveSubsystem.getInstance()),
            new WaitCommand(0.05),
            // new AutoScoreCommand(),
            path1,
            new WaitCommand(0.6),
            path2
            // new AutoScoreCubePuke()
        );
    }
}

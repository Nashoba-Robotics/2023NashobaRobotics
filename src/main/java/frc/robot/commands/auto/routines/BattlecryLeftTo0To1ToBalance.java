package frc.robot.commands.auto.routines;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.intakescore.AutoScoreCubeCommand;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.commands.intake.IntakeCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BattlecryLeftTo0To1ToBalance extends SequentialCommandGroup{
    public BattlecryLeftTo0To1ToBalance(){
        Map<String, Command> map = new HashMap<>();

        map.put("Start Intake", new IntakeCubeCommand());
        map.put("Stop Intake", new InstantCommand(
            () -> {
                ArmSubsystem.getInstance().setDefaultAcceleration();
                ArmSubsystem.getInstance().pivot(0);
                GrabberSubsystem.getInstance().set(-0.1);
            },
            ArmSubsystem.getInstance(),
            GrabberSubsystem.getInstance()
        ));
        map.put("Prep Score Angle", new InstantCommand(() -> ArmSubsystem.getInstance().pivot(-22 * Constants.TAU/360), ArmSubsystem.getInstance()));

        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Battlecry-Smooth-2.5-Climb", new PathConstraints(4, 3));

        FollowPathWithEvents path1 = new FollowPathWithEvents(
            new FollowPathCommand(path.get(0)),
             path.get(0).getMarkers(), 
             map
        );

        FollowPathWithEvents path2 = new FollowPathWithEvents(
            new FollowPathCommand(path.get(1)),
             path.get(1).getMarkers(), 
             map
        );

        addCommands(
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().zeroWrist();
                SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
            }, GrabberSubsystem.getInstance(), SwerveDriveSubsystem.getInstance()),
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().resetOdometry(PathPlannerTrajectory.transformTrajectoryForAlliance(
                    path.get(0),
                    DriverStation.getAlliance()).getInitialHolonomicPose());
            }, SwerveDriveSubsystem.getInstance()),
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().resetOdometry(PathPlannerTrajectory.transformTrajectoryForAlliance(
                    path.get(0),
                    DriverStation.getAlliance()).getInitialHolonomicPose());
            }, SwerveDriveSubsystem.getInstance()),
            new WaitCommand(0.1),

            path1,
            new AutoScoreCubeCommand(),
            path2,
            null
        );
    }
}

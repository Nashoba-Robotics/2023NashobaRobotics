package frc.robot.commands.auto.routines;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RightTo3ToScoreAuto extends SequentialCommandGroup{
    public RightTo3ToScoreAuto(){
        Map<String, Command> map = new HashMap<>();
        map.put("Start Intake", new IntakeCommand(true));
        map.put("Stop Intake", new InstantCommand(
            () -> {
                ArmSubsystem.getInstance().pivot(0);
                GrabberSubsystem.getInstance().set(-0.1);
            },
            ArmSubsystem.getInstance(),
            GrabberSubsystem.getInstance()
        ));
        PathPlannerTrajectory path = PathPlanner.loadPath("BLUE-rightC-3-rightA", new PathConstraints(2, 2));
        
        FollowPathWithEvents command = new FollowPathWithEvents(
            new FollowPathCommand(path),
            path.getMarkers(),
            map);


        addCommands(
            new InstantCommand(() -> GrabberSubsystem.getInstance().zeroWrist(), GrabberSubsystem.getInstance()),
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().setGyro(0)),
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().resetOdometry(path.getInitialHolonomicPose());
            }, SwerveDriveSubsystem.getInstance()
            ),
            new AutoScoreCommand(),
            command,
            new AutoScoreCommand()
        );
    }
}

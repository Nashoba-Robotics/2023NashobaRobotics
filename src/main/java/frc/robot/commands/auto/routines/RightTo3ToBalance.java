package frc.robot.commands.auto.routines;

import java.util.HashMap;
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
import frc.robot.commands.IntakeCubeCommand;
import frc.robot.commands.auto.balance.AutoBalanceCommand;
import frc.robot.commands.auto.balance.routine.backToBalance;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RightTo3ToBalance extends SequentialCommandGroup {
    
    public RightTo3ToBalance() {
        Map<String, Command> map = new HashMap<>();
        map.put("Start Intake", new IntakeCubeCommand());
        map.put("Stop Intake", new InstantCommand(
            () -> {
                ArmSubsystem.getInstance().pivot(0);
                GrabberSubsystem.getInstance().set(-0.1);
            },
            ArmSubsystem.getInstance(),
            GrabberSubsystem.getInstance()
        ));
        PathPlannerTrajectory path = PathPlanner.loadPath("rightC-3-climb", new PathConstraints(2, 2));

        FollowPathWithEvents command = new FollowPathWithEvents(
            new FollowPathCommand(path),
            path.getMarkers(),
            map);
            
        addCommands(
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().zeroWrist();
                ArmSubsystem.getInstance().resetPivotNU();
                SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
            }, GrabberSubsystem.getInstance(), ArmSubsystem.getInstance()),
            new WaitCommand(0.1),
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().resetOdometry(PathPlannerTrajectory.transformTrajectoryForAlliance(
                    path,
                    DriverStation.getAlliance()).getInitialHolonomicPose());
            }, SwerveDriveSubsystem.getInstance()),
            new WaitCommand(0.1),
            new AutoScoreCommand(),
            command,
            new backToBalance(),
            new AutoBalanceCommand()
        );
    }

}

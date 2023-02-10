package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoPaths;
import frc.robot.Constants;
import frc.robot.commands.auto.FollowPathCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TestAutoCommand extends SequentialCommandGroup {
    
    public TestAutoCommand() {
        addCommands(
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2), SwerveDriveSubsystem.getInstance()),
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0))), SwerveDriveSubsystem.getInstance()),
            new FollowPathCommand(AutoPaths.testTrajectory)
        );
    }
}

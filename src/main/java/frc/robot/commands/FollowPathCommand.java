package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FollowPathCommand extends SequentialCommandGroup {
 
    public FollowPathCommand(Trajectory trajectory) {

        SwerveControllerCommand swerveController = new SwerveControllerCommand(
            trajectory,
            SwerveDriveSubsystem.getInstance()::getPose,
            Constants.Swerve.KINEMATICS,
            new HolonomicDriveController(
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0))
                ),
            SwerveDriveSubsystem.getInstance()::setStates,
            SwerveDriveSubsystem.getInstance());

    }

}

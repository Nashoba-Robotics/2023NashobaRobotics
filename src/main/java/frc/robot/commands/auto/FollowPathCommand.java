package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FollowPathCommand extends SequentialCommandGroup {
 
    public FollowPathCommand(PathPlannerTrajectory trajectory) {

        PIDController xController = new PIDController(Constants.Swerve.Auto.P_X, 0, Constants.Swerve.Auto.D_X);
        PIDController yController = new PIDController(Constants.Swerve.Auto.P_Y, 0, Constants.Swerve.Auto.D_Y);
        PIDController thetaController = new PIDController(Constants.Swerve.Auto.P_THETA, 0, 0);
        thetaController.enableContinuousInput(-Constants.TAU/2, Constants.TAU/2);

        PPSwerveControllerCommand swerveController = new PPSwerveControllerCommand(
            trajectory,
            SwerveDriveSubsystem.getInstance()::getPose,
            Constants.Swerve.KINEMATICS,
            xController,
            yController,
            thetaController,
            SwerveDriveSubsystem.getInstance()::setStates,
            SwerveDriveSubsystem.getInstance()
            );

        // SwerveControllerCommand swerveController = new SwerveControllerCommand(
        //     trajectory,
        //     SwerveDriveSubsystem.getInstance()::getPose,
        //     Constants.Swerve.KINEMATICS,
        //     controller,
        //     SwerveDriveSubsystem.getInstance()::setStates,
        //     SwerveDriveSubsystem.getInstance());

            addCommands(
                new InstantCommand(() -> {SwerveDriveSubsystem.getInstance().resetOdometry(trajectory.getInitialHolonomicPose());}, SwerveDriveSubsystem.getInstance()),
                swerveController,
                new InstantCommand(() -> {SwerveDriveSubsystem.getInstance().set(0, 0, 0);}, SwerveDriveSubsystem.getInstance())
            );

    }

}

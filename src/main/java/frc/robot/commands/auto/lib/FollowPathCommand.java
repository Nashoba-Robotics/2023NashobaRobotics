package frc.robot.commands.auto.lib;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/*
    Takes a PathPlannerTrajectory
    Moves swerve drive base along the trajectory
*/
public class FollowPathCommand extends SequentialCommandGroup {
 
    public FollowPathCommand(PathPlannerTrajectory trajectory) {
        //PID controllers for each axis of control
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
            true,
            SwerveDriveSubsystem.getInstance()
            );

            addCommands(
                new InstantCommand(() -> {SwerveDriveSubsystem.getInstance().resetOdometry(trajectory.getInitialHolonomicPose());}, SwerveDriveSubsystem.getInstance()),
                swerveController
            );

    }

}
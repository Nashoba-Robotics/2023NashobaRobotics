package frc.robot.commands.auto.move;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

public class DriveToCommand extends CommandBase {

    private Translation2d endPos;
    private Timer timer;
    private long startTime;
    Rotation2d endHolonomic;

    private Command currCommand;
    PathPlannerTrajectory trajectory;
    
    public DriveToCommand(Translation2d endPos) {
        // addRequirements(SwerveDriveSubsystem.getInstance());
        this.endPos = endPos;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        CandleSubsystem.getInstance().set(CandleState.FORDIANI);
        startTime = System.currentTimeMillis();

        Pose2d pose = new Pose2d(LimelightSubsystem.getInstance().getRobotPose().getTranslation(), Rotation2d.fromRadians(SwerveDriveSubsystem.getInstance().getGyroAngle()));
        SwerveDriveSubsystem.getInstance().resetOdometry(pose);

        endHolonomic = Rotation2d.fromRadians(
            SwerveDriveSubsystem.getInstance().getGyroAngle() > Constants.TAU/4 || SwerveDriveSubsystem.getInstance().getGyroAngle() < -Constants.TAU/4 ?
            Constants.TAU/2 :
            0
        );

        double targetX = endPos.getX();
        double x = pose.getX();

        double targetY = endPos.getY();
        double y = pose.getY();

        double startHeading = Math.atan2((targetY-y), (targetX-x));
        double endHeading = startHeading;

        //8.02
        // if(DriverStation.getAlliance() == Alliance.Red) {
        //     endPos = new Translation2d(endPos.getX(), 8.02 - endPos.getY());
        // }

        trajectory = PathPlanner.generatePath(
            new PathConstraints(2, 1.5), 
            List.of(
                new PathPoint(SwerveDriveSubsystem.getInstance().getPose().getTranslation(), Rotation2d.fromRadians(startHeading), SwerveDriveSubsystem.getInstance().getPose().getRotation(), SwerveDriveSubsystem.getInstance().getVelocity()),
                new PathPoint(endPos, Rotation2d.fromRadians(endHeading), endHolonomic)
            )
        );

        currCommand = new FollowPathCommand(trajectory);

        CommandScheduler.getInstance().schedule(currCommand);

        timer.reset();
        timer.start();
        SmartDashboard.putBoolean("APB", false);
    }

    @Override
    public void execute() {
        

        if(LimelightSubsystem.getInstance().isTarget()) {
            

            timer.reset();
            SmartDashboard.putBoolean("APB", true);
        }
        SmartDashboard.putNumber("timer", timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        CandleSubsystem.getInstance().set(CandleState.ENABLED);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > 2500;
    }

}

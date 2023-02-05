package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class CameraTestCommand extends CommandBase{
    
    public CameraTestCommand() {
        LimelightSubsystem.getInstance().setPipeline(1);
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        // LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.APRIL_TAG_PIPELINE);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("x", SwerveDriveSubsystem.getInstance().getPose().getX());
        SmartDashboard.putNumber("y", SwerveDriveSubsystem.getInstance().getPose().getY());
        SwerveDriveSubsystem.getInstance().resetOdometry(LimelightSubsystem.getInstance().getRobotPose());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

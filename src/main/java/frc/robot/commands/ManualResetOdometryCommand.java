package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ManualResetOdometryCommand extends CommandBase {
    
    public ManualResetOdometryCommand() {
        SmartDashboard.putNumber("XReset", 0);
        SmartDashboard.putNumber("YReset", 0);
    }

    @Override
    public void initialize() {
        SwerveDriveSubsystem.getInstance().resetOdometry(
            new Pose2d(
                SmartDashboard.getNumber("XReset", 0),
                SmartDashboard.getNumber("YReset", 0),
                Rotation2d.fromRadians(SwerveDriveSubsystem.getInstance().getGyroAngle())
            )
        );
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

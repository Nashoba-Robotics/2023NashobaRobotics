package frc.robot.commands.auto;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MoveBackCommand extends CommandBase {
    
    long startTime;

    public MoveBackCommand() {
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SwerveDriveSubsystem.getInstance().set(0.2, 0, 0);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > 5000;
    }

}

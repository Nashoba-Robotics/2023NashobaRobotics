package frc.robot.commands.score;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ScoreConeCommand extends CommandBase {
    private long startTime;    

    public ScoreConeCommand() {
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        double gyroAngle = SwerveDriveSubsystem.getInstance().getGyroAngle();
        boolean scoreFront = gyroAngle > Constants.TAU/4 || gyroAngle < -Constants.TAU/4;

        int multiplier = scoreFront ? 1 : -1;


        GrabberSubsystem.getInstance().setCurrentLimit(40);
        ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        ArmSubsystem.getInstance().setDefaultAcceleration();
        startTime = System.currentTimeMillis();


        double angleChange = DriverStation.isAutonomous() ? 2 * Constants.TAU/360 : 2 * Constants.TAU/360;
        ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getAngle() + angleChange * multiplier);
        GrabberSubsystem.getInstance().orientPos(-2 * multiplier);
        ArmSubsystem.getInstance().extendNU(1000);
        GrabberSubsystem.getInstance().set(0.1);
    }

    @Override
    public void execute() {
        
        
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().orient(0);
        ArmSubsystem.getInstance().stop();
        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(0);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > 2000;
    }

}

package frc.robot.commands.score;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ScoreCubeCommand extends CommandBase {
    private long startTime;    

    public ScoreCubeCommand() {
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


        // double angleChange = DriverStation.isAutonomous() ? 3 * Constants.TAU/360 : 2 * Constants.TAU/360;
        // ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getAngle() + angleChange * multiplier);
        // ArmSubsystem.getInstance().extendNU(1000);
        GrabberSubsystem.getInstance().set(-0.4, 0.4);
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
        return false;
    }

}

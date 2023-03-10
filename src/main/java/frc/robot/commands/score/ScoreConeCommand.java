package frc.robot.commands.score;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ScoreConeCommand extends CommandBase {
    private long startTime;    
    private boolean retractFirst;
    private double retractTarget1;

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


        double angleChange = DriverStation.isAutonomous() ? 3 * Constants.TAU/360 : 0 * Constants.TAU/360;
        ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getAngle() + angleChange * multiplier);
        GrabberSubsystem.getInstance().orientPos(4 * multiplier);
        double currentExtend = ArmSubsystem.getInstance().getExtendNU();
        ArmSubsystem.getInstance().extendNU(currentExtend-13_000);
        GrabberSubsystem.getInstance().set(0.1);
    }

    @Override
    public void execute() {
        
        
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().orient(0);
        // ArmSubsystem.getInstance().stop();
        ArmSubsystem.getInstance().setPivotCruiseVelocity(50_000);
        ArmSubsystem.getInstance().setPivotAcceleration(50_000);
        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(0);
    }

    @Override
    public boolean isFinished() {
        if(DriverStation.isAutonomous()) return System.currentTimeMillis() - startTime > 500;
        return System.currentTimeMillis() - startTime > 2000;
    }

}

package frc.robot.commands.score;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Grabber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ScoreConeCommand extends CommandBase {
    private boolean low;
    private long startTime;    

    public ScoreConeCommand() {
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        double gyroAngle = SwerveDriveSubsystem.getInstance().getGyroAngle();
        boolean scoreFront = gyroAngle > Constants.TAU/4 || gyroAngle < -Constants.TAU/4;

        // GrabberSubsystem.getInstance().setCurrentLimit(true);

        int multiplier = scoreFront ? 1 : -1;

        // GrabberSubsystem.getInstance().setCurrentLimit(40);
        ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        ArmSubsystem.getInstance().setDefaultAcceleration();

        ArmSubsystem.getInstance().setPivotCruiseVelocity(85_000);
        ArmSubsystem.getInstance().setPivotAcceleration(60_000);

        startTime = System.currentTimeMillis();

        low = Math.abs(ArmSubsystem.getInstance().getAngle()) > Constants.TAU/4;
        if(low){
            // GrabberSubsystem.getInstance().setCurrentLimit(50);
            GrabberSubsystem.getInstance().setCurrentLimit(false);
            GrabberSubsystem.getInstance().outtake(Constants.Grabber.LOW_CONE_RELEASE_SPEED);
            ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getAngle());
        } 
        else{
            GrabberSubsystem.getInstance().setCurrentLimit(true);
            double angleChange = DriverStation.isAutonomous() ? Constants.Arm.AUTO_DUNK_ANGLE : Constants.Arm.TELEOP_DUNK_ANGLE;
            ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getAngle() + angleChange * multiplier);
            GrabberSubsystem.getInstance().orientPos(Constants.Grabber.SCORE_CONE_NU * multiplier);
            double currentExtend = ArmSubsystem.getInstance().getExtendNU();
            ArmSubsystem.getInstance().extendNU(currentExtend-Constants.Arm.RETRACT_NU);
            GrabberSubsystem.getInstance().set(Constants.Grabber.CONE_RELEASE_SPEED);
        }
    }

    @Override
    public void execute() {
        
        
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().setCurrentLimit(true);
        GrabberSubsystem.getInstance().set(0);
        GrabberSubsystem.getInstance().orient(0);
        if(DriverStation.isAutonomous()){   //If it pivots faster when retracting, it will tip -> Don't want that in auto
            ArmSubsystem.getInstance().setPivotAcceleration(25_000);
            ArmSubsystem.getInstance().setPivotCruiseVelocity(30_000);

            ArmSubsystem.getInstance().setExtendCruiseVelocity(40_000);
            ArmSubsystem.getInstance().setExtendAcceleration(20_000);
        }
        else{   //Go fast in teleop VROOM
            ArmSubsystem.getInstance().setPivotCruiseVelocity(50_000);
            ArmSubsystem.getInstance().setPivotAcceleration(50_000);
        }
        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(3_000);
    }

    @Override
    public boolean isFinished() {
        if(low) return System.currentTimeMillis() - startTime > 5000;
        
        return System.currentTimeMillis() - startTime > 400;
    }

}

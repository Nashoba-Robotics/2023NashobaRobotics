package frc.robot.commands.score;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ScoreConeCommand extends CommandBase {
    private TargetLevel targetLevel;
    private int startTime;

    public ScoreConeCommand(TargetLevel targetLevel) {
        this.targetLevel = targetLevel;
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        startTime = (int)System.currentTimeMillis();
        switch(targetLevel) {
            case HIGH:
                ArmSubsystem.getInstance().pivot(Constants.Arm.HIGH_ANGLE);
                GrabberSubsystem.getInstance().orient(Constants.Grabber.HIGH_ANGLE);
            break;
            case MID:
                ArmSubsystem.getInstance().pivot(Constants.Arm.MID_ANGLE);
                GrabberSubsystem.getInstance().orient(Constants.Grabber.MID_ANGLE);
            break;
            case LOW:
                ArmSubsystem.getInstance().pivot(Constants.Arm.LOW_ANGLE);
                GrabberSubsystem.getInstance().orient(Constants.Grabber.LOW_ANGLE);
            break;
        }
      
    }

    @Override
    public void execute() {
        switch(targetLevel) {
            case HIGH:
               if(Math.abs(ArmSubsystem.getInstance().getAngle() - Constants.Arm.HIGH_ANGLE) < Constants.Arm.ERROR_ANGLE &&
                  Math.abs(GrabberSubsystem.getInstance().getOrientation() - Constants.Arm.HIGH_ANGLE) < Constants.Grabber.ERROR_ANGLE) {
                 GrabberSubsystem.getInstance().score();
               } 
            break;
            case MID:
               if(Math.abs(ArmSubsystem.getInstance().getAngle() - Constants.Arm.MID_ANGLE) < Constants.Arm.ERROR_ANGLE &&
                  Math.abs(GrabberSubsystem.getInstance().getOrientation() - Constants.Arm.MID_ANGLE) < Constants.Grabber.ERROR_ANGLE) {
                  GrabberSubsystem.getInstance().score();
               } 
            break;
            case LOW:
                if(Math.abs(ArmSubsystem.getInstance().getAngle() - Constants.Arm.LOW_ANGLE) < Constants.Arm.ERROR_ANGLE &&
                   Math.abs(GrabberSubsystem.getInstance().getOrientation() - Constants.Arm.LOW_ANGLE) < Constants.Grabber.ERROR_ANGLE) {
                  GrabberSubsystem.getInstance().score();
                } 
            break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().pivot(0);
    }

    @Override
    public boolean isFinished() {
       return System.currentTimeMillis() - startTime > 5000;
    }

}

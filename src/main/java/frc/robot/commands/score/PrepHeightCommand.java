package frc.robot.commands.score;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.Constants.Field.TargetLevel;


public class PrepHeightCommand extends CommandBase {
    private TargetLevel targetLevel;

    double lastPos;
    boolean joystick0;
    boolean gotToStart;

    public PrepHeightCommand(TargetLevel targetLevel) {
        this.targetLevel = targetLevel;
        addRequirements(GrabberSubsystem.getInstance(), ArmSubsystem.getInstance());
    }

    public void initialize() {
        switch(targetLevel) {
            case HIGH: 
             GrabberSubsystem.getInstance().orientPos(-3);
             ArmSubsystem.getInstance().pivot(Constants.TAU / 6);
             ArmSubsystem.getInstance().extendNU(31_629);
             break;
            case MID: 
             GrabberSubsystem.getInstance().orientPos(-3);
             ArmSubsystem.getInstance().pivot(65 * Constants.TAU / 360);
             break;
           // case LOW: 
           //  ArmSubsystem.getInstance().extend(0);
        }
        gotToStart = false;
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("manual", gotToStart);
        SmartDashboard.putNumber("Arm nu", ArmSubsystem.getInstance().getPos());
        if(!gotToStart && Math.abs(ArmSubsystem.getInstance().getPos() - 31_000) < 500) gotToStart = true;
        if(gotToStart) {
            double y = RobotContainer.operatorController.getThrottle() * 0.3;
            y = Math.abs(y) < 0.1 ? 0 : (y-0.1)/0.9;    //Put deadzone in Constants
            if(y == 0){ // If there isn't any input, maintain the position
                ArmSubsystem.getInstance().holdArm();
                if(!joystick0){
                    joystick0 = true;
                    lastPos = ArmSubsystem.getInstance().getPos();
                }
                ArmSubsystem.getInstance().extendNU(lastPos);
            }
            else{
                ArmSubsystem.getInstance().set(-y*0.3);
                joystick0 = false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    public boolean isFinished() {
        return false;
    }
}

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
    TargetLevel targetLevel;
    double targetPos;

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
             ArmSubsystem.getInstance().pivot(Constants.Arm.HIGH_ANGLE);
             ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU);
             targetPos = Constants.Arm.HIGH_EXTEND_NU;
             break;
            case MID: 
             GrabberSubsystem.getInstance().orientPos(-3);
             ArmSubsystem.getInstance().pivot(Constants.Arm.MID_ANGLE);
             ArmSubsystem.getInstance().extendNU(Constants.Arm.MID_EXTEND_NU);
             targetPos = Constants.Arm.MID_EXTEND_NU;
             break;
           case LOW: 
            ArmSubsystem.getInstance().extend(Constants.Arm.LOW_ANGLE);
            ArmSubsystem.getInstance().extendNU(Constants.Arm.LOW_EXTEND_NU);
            targetPos = Constants.Arm.LOW_EXTEND_NU;
            break;
        }
        gotToStart = false;
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("manual", gotToStart);
        SmartDashboard.putNumber("Arm nu", ArmSubsystem.getInstance().getPos());
        if(!gotToStart && Math.abs(ArmSubsystem.getInstance().getPos() - targetPos) < 500) gotToStart = true;
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
